#include "CAN_FDAnalyzer.h"
#include "CAN_FDAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

CAN_FDAnalyzer::CAN_FDAnalyzer()
:	Analyzer2(),  
	mSettings( new CAN_FDAnalyzerSettings() ),
	mSimulationInitilized( false )
{
	SetAnalyzerSettings( mSettings.get() );
}

CAN_FDAnalyzer::~CAN_FDAnalyzer()
{
	KillThread();
}

void CAN_FDAnalyzer::SetupResults()
{
	mResults.reset( new CAN_FDAnalyzerResults( this, mSettings.get() ) );
	SetAnalyzerResults( mResults.get() );
	mResults->AddChannelBubblesWillAppearOn( mSettings->mInputChannel );
}

void CAN_FDAnalyzer::WorkerThread()
{
	mSampleRateHz = GetSampleRate();
	mCAN_FD = GetAnalyzerChannelData(mSettings->mInputChannel);

	InitSampleOffsets();

	/* Get to an inter-frame gap at the slow timing */
	WaitFor7RecessiveBits();

	//now let's pull in the frames, one at a time.
	for (; ; )
	{
		if (mCAN_FD->GetBitState() == mSettings->Recessive())
			mCAN_FD->AdvanceToNextEdge();

		//we're at the first DOMINANT edge of the frame
		GetRawFrame();
		AnalyzeRawFrame();

		if (mCanError == true)
		{
			Frame frame;
			frame.mStartingSampleInclusive = mErrorStartingSample;
			frame.mEndingSampleInclusive = mErrorEndingSample;
			frame.mType = CanError;
			mResults->AddFrame(frame);
			mResults->CancelPacketAndStartNewPacket();
		}

		U32 count = (U32)mCanMarkers.size();
		for (U32 i = 0; i < count; i++)
		{
			if (mCanMarkers[i].mType == Standard)
				mResults->AddMarker(mCanMarkers[i].mSample, AnalyzerResults::Dot, mSettings->mInputChannel);
			else
				mResults->AddMarker(mCanMarkers[i].mSample, AnalyzerResults::ErrorX, mSettings->mInputChannel);
		}

		mResults->CommitResults();
		ReportProgress(mCAN_FD->GetSampleNumber());
		CheckIfThreadShouldExit();

		if (mCanError == true)
		{
			WaitFor7RecessiveBits();
		}
	}
}

void CAN_FDAnalyzer::InitSampleOffsets()
{
	mSampleOffsets.resize(1440);

	mBitRateFactor = (int)mSettings->mBitRateData / (int)mSettings->mBitRateHdr;

	double samples_per_bit = double(mSampleRateHz) / double(mSettings->mBitRateData);
	double samples_behind = 0.0;

	U32 increment = U32((samples_per_bit * .5) + samples_behind);
	samples_behind = (samples_per_bit * .5) + samples_behind - double(increment);

	mSampleOffsets[0] = increment;
	U32 current_offset = increment;

	for (U32 i = 1; i < 1440; i++)
	{
		U32 increment = U32(samples_per_bit + samples_behind);
		samples_behind = samples_per_bit + samples_behind - double(increment);
		current_offset += increment;
		mSampleOffsets[i] = current_offset;
	}

	mNumSamplesIn7Bits = U32(samples_per_bit * 7.0 * mBitRateFactor);   /* This bit time is at the slow header bit rate */
}

void CAN_FDAnalyzer::WaitFor7RecessiveBits()
{
	if (mCAN_FD->GetBitState() == mSettings->Dominant())
		mCAN_FD->AdvanceToNextEdge();

	for (; ; )
	{
		if (mCAN_FD->WouldAdvancingCauseTransition(mNumSamplesIn7Bits) == false)
			return;

		mCAN_FD->AdvanceToNextEdge();
		mCAN_FD->AdvanceToNextEdge();
	}
}

void CAN_FDAnalyzer::GetRawFrame()
{
	mCanError = false;
	mRecessiveCount = 0;
	mDominantCount = 0;
	mRawBitResults.clear();

	if (mCAN_FD->GetBitState() != mSettings->Dominant())
		AnalyzerHelpers::Assert("GetFrameOrError assumes we start DOMINANT");

	mStartOfFrame = mCAN_FD->GetSampleNumber();

	U32 i = 0;
	U32 j = 0;
	//what we're going to do now is capture a sequence up until we get 7 recessive bits (at slow timing) in a row.
	for (; ; )
	{
		if (i > 1440)
		{
			//we are in garbage data most likely, lets get out of here.
			return;
		}

		mCAN_FD->AdvanceToAbsPosition(mStartOfFrame + mSampleOffsets[i]);
		i++;

		if (mCAN_FD->GetBitState() == mSettings->Dominant())
		{
			//the bit is DOMINANT
			mDominantCount++;
			mRecessiveCount = 0;
			mRawBitResults.push_back(mSettings->Dominant());

			if (mDominantCount == (6 * mBitRateFactor))
			{
				//we have detected an error.

				mCanError = true;
				mErrorStartingSample = mStartOfFrame + mSampleOffsets[i - (5 * mBitRateFactor)];
				mErrorEndingSample = mStartOfFrame + mSampleOffsets[i];

				//don't use any of these error bits in analysis.
				for (j = 0; j < (6 * mBitRateFactor); j++)
				{
					mRawBitResults.pop_back();
				}

				mNumRawBits = (U32)mRawBitResults.size();

				break;
			}
		}
		else
		{
			//the bit is RECESSIVE
			mRecessiveCount++;
			mDominantCount = 0;
			mRawBitResults.push_back(mSettings->Recessive());

			if (mRecessiveCount == (7 * mBitRateFactor))
			{
				//we're done.
				break;
			}
		}
	}

	mNumRawBits = (U32)mRawBitResults.size();
}

void CAN_FDAnalyzer::AnalyzeRawFrame()
{
	BitState bit;
	U8 frametype;
	U64 last_sample;

	mBaudSwitch = false;  /* Assume no baud rate switch required */
	mCheckingBRS = false; /* Not checking the BRS bit at present */

	UnstuffRawFrameBit(bit, last_sample, true);  //grab the start bit, and reset everything.
	mArbitrationField.clear();
	mControlField.clear();
	mDataField.clear();
	mCrcFieldWithoutDelimiter.clear();
	mAckField.clear();

	bool done;

	/* Identifier section comes immediately after start bit */

	mIdentifier = 0;
	for (U32 i = 0; i < 11; i++)
	{
		mIdentifier <<= 1;
		BitState bit;
		done = UnstuffRawFrameBit(bit, last_sample);
		if (done == true)
			return;
		mArbitrationField.push_back(bit);

		if (bit == mSettings->Recessive())
			mIdentifier |= 1;
	}

	//ok, the next three bits will let us know if this is 11-bit or 29-bit can.  If it's 11-bit, then it'll also tell us if this is a remote frame request or not.

	BitState rtr_rrs;
	done = UnstuffRawFrameBit(rtr_rrs, last_sample);
	if (done == true)
		return;

	BitState ide;
	done = UnstuffRawFrameBit(ide, last_sample);
	if (done == true)
		return;

	/* If ide is dominant, then this is an 11-bit header, else it is a 29-bit */

	if (ide == mSettings->Dominant())
	{
		//11-bit CAN

		BitState fdf_res;
		done = UnstuffRawFrameBit(fdf_res, last_sample);
		if (done == true)
			return;

		/* fdf_res bit is the key to recognising whether the frame is standard CAN or CAN-FD */
		/* This bit is dominant 0 on classic CAN, and recessive 1 on CAN-FD */

		if (fdf_res == mSettings->Dominant())
		{
			/* Standard 11-bit CAN */
			frametype = IdentifierField;
			mStandardCan = true;
			mStandardCanFD = false;

			Frame frame;
			frame.mStartingSampleInclusive = mStartOfFrame + mSampleOffsets[1];
			frame.mEndingSampleInclusive = last_sample;
			frame.mType = frametype;

			if (rtr_rrs == mSettings->Recessive()) //since this is 11-bit Standard CAN, we know that rtr_rrs is the RTR bit
			{
				mRemoteFrame = true;
				frame.mFlags = REMOTE_FRAME;
			}
			else
			{
				mRemoteFrame = false;
				frame.mFlags = 0;
			}

			frame.mData1 = mIdentifier;
			mResults->AddFrame(frame);
		}
		else
		{
			/* Standard CAN-FD frame */

			frametype = FDIdentifier;
			mStandardCan = false;
			mStandardCanFD = true;

			Frame frame;
			frame.mStartingSampleInclusive = mStartOfFrame + mSampleOffsets[1];
			frame.mEndingSampleInclusive = last_sample;
			frame.mType = frametype;

			mRemoteFrame = false;    /* Always a data frame */
			frame.mFlags = 0;

			frame.mData1 = mIdentifier;
			mResults->AddFrame(frame);

			/* 3 additional bits before control frame */

			/* Reserved bit for future use - ignored */
			BitState fdres;
			done = UnstuffRawFrameBit(fdres, last_sample);
			if (done == true)
				return;

			mCheckingBRS = true;

			/* Baud Rate Switch - data section of packet sent at higher rate if this bit is recessive*/
			BitState brs;
			done = UnstuffRawFrameBit(brs, last_sample);
			if (done == true)
				return;

			mCheckingBRS = false;

			/* If using flexible data rate, fast bit timing starts now */
			
			/* Error state indicator */
			BitState esi;
			done = UnstuffRawFrameBit(esi, last_sample);
			if (done == true)
				return;
		}
	}
	else
	{
		//29-bit CAN

		//get the next 18 address bits.
		for (U32 i = 0; i < 18; i++)
		{
			mIdentifier <<= 1;

			BitState bit;
			done = UnstuffRawFrameBit(bit, last_sample);
			if (done == true)
				return;
			mArbitrationField.push_back(bit);

			if (bit == mSettings->Recessive())
				mIdentifier |= 1;
		}

		//get the RTR bit
		BitState rtr;
		done = UnstuffRawFrameBit(rtr, last_sample);
		if (done == true)
			return;

		/* Get r0_fdf - this determines whether this is an FD frame or not */
		BitState r0_fdf;
		done = UnstuffRawFrameBit(r0_fdf, last_sample);
		if (done == true)
			return;

		if (r0_fdf == mSettings->Dominant())
		{
			/* Standard 29-bit CAN frame */
			frametype = IdentifierFieldEx;
			BitState r1;
			done = UnstuffRawFrameBit(r1, last_sample);
			if (done == true)
				return;

			Frame frame;
			frame.mStartingSampleInclusive = mStartOfFrame + mSampleOffsets[1];
			frame.mEndingSampleInclusive = last_sample;
			frame.mType = frametype;

			if (rtr == mSettings->Recessive())
			{
				mRemoteFrame = true;
				frame.mFlags = REMOTE_FRAME;
			}
			else
			{
				mRemoteFrame = false;
				frame.mFlags = 0;
			}

			frame.mData1 = mIdentifier;
			mResults->AddFrame(frame);
		}
		else
		{
			/* 29-bit extended CAN-FD frame */
			frametype = FDIdentifierEx;
			Frame frame;
			frame.mStartingSampleInclusive = mStartOfFrame + mSampleOffsets[1];
			frame.mEndingSampleInclusive = last_sample;
			frame.mType = frametype;

			mRemoteFrame = false;    /* Always a data frame */
			frame.mFlags = 0;

			frame.mData1 = mIdentifier;
			mResults->AddFrame(frame);

			/* 3 additional bits in CAN-FD prior to control frame */

			/* Reserved bit for future use - ignored */
			BitState fdres;
			done = UnstuffRawFrameBit(fdres, last_sample);
			if (done == true)
				return;

			mCheckingBRS = true;

			/* Baud Rate Switch - rest of frame sent at higher rate if this bit is recessive*/
			BitState brs;
			done = UnstuffRawFrameBit(brs, last_sample);
			if (done == true)
				return;

			mCheckingBRS = false;

			/* If using flexible data rate, fast bit timing starts now */

			/* Error state indicator */
			BitState esi;
			done = UnstuffRawFrameBit(esi, last_sample);
			if (done == true)
				return;
		}
	}

	/* 4 control bits come next - defines the Data Length Code (DLC) for this packet */

	U32 mask = 0x8;
	U32 dlc = 0;
	U64 first_sample = 0;
	for (U32 i = 0; i < 4; i++)
	{
		BitState bit;
		if (i == 0)
			done = UnstuffRawFrameBit(bit, first_sample);
		else
			done = UnstuffRawFrameBit(bit, last_sample);

		if (done == true)
			return;

		mControlField.push_back(bit);

		if (bit == mSettings->Recessive())
			dlc |= mask;

		mask >>= 1;
	}

	if ((frametype == FDIdentifierEx) || (frametype == FDIdentifier))
	{
		/* CAN-FD frame - DLC is extended and is not equal to number of bytes in packet in all cases */
		if (dlc < 9)
		{
			mNumDataBytes = dlc;
		}
		else if (dlc == 9)
		{
			mNumDataBytes = 12;
		}
		else if (dlc == 10)
		{
			mNumDataBytes = 16;
		}
		else if (dlc == 11)
		{
			mNumDataBytes = 20;
		}
		else if (dlc == 12)
		{
			mNumDataBytes = 24;
		}
		else if (dlc == 13)
		{
			mNumDataBytes = 32;
		}
		else if (dlc == 14)
		{
			mNumDataBytes = 48;
		}
		else
		{
			mNumDataBytes = 64;
		}
	}
	else
	{
		/* Standard CAN frame - supplied DLC is the exact number of bytes in the frame */
		mNumDataBytes = dlc;
	}

	Frame frame;
	frame.mStartingSampleInclusive = first_sample;
	frame.mEndingSampleInclusive = last_sample;
	frame.mType = ControlField;
	frame.mData1 = mNumDataBytes;
	frame.mFlags = 0;
	mResults->AddFrame(frame);

	U32 num_bytes = mNumDataBytes;

	if (mRemoteFrame == true)
		num_bytes = 0; //ignore the num_bytes if this is a remote frame.

	/* Data section. Get and unpack pre-determined number of bytes from this packet */

	for (U32 i = 0; i < num_bytes; i++)
	{
		U32 data = 0;
		U32 mask = 0x80;
		for (U32 j = 0; j < 8; j++)
		{
			BitState bit;
			if (j == 0)
				done = UnstuffRawFrameBit(bit, first_sample);
			else
				done = UnstuffRawFrameBit(bit, last_sample);

			if (done == true)
				return;

			if (bit == mSettings->Recessive())
				data |= mask;

			mask >>= 1;

			mDataField.push_back(bit);
		}
		Frame frame;
		frame.mStartingSampleInclusive = first_sample;
		frame.mEndingSampleInclusive = last_sample;
		frame.mType = DataField;
		frame.mData1 = data;
		mResults->AddFrame(frame);
	}

	/* End of data section */

	U32 crc_bytes;

	if ((frametype == FDIdentifierEx) || (frametype == FDIdentifier))
	{
		/* 6 additional bits in CAN-FD identifier frame prior to CRC Section */

	    /* Fixed Stuff bit 1 */
		BitState fsb1;
		done = UnstuffFixedStuffBit(fsb1, last_sample);
		if (done == true)
			return;

		/* Stuff count bit 2 */
		BitState sc_bit2;
		done = UnstuffRawFrameBit(sc_bit2, last_sample);
		if (done == true)
			return;

		/* Stuff count bit 1 */
		BitState sc_bit1;
		done = UnstuffRawFrameBit(sc_bit1, last_sample);
		if (done == true)
			return;

		/* Stuff count bit 0 */
		BitState sc_bit0;
		done = UnstuffRawFrameBit(sc_bit0, last_sample);
		if (done == true)
			return;

		/* Stuff bits parity */
		BitState stuff_parity;
		done = UnstuffRawFrameBit(stuff_parity, last_sample);
		if (done == true)
			return;

		/* Fixed Stuff bit 2 */
		BitState fsb2;
		done = UnstuffFixedStuffBit(fsb2, last_sample);
		if (done == true)
			return;

		/* CRC section for CAN-FD */

		/* CRC length depends on packet data length. Stuffing bits are in fixed, known positions */
		if (mNumDataBytes >= 20)
		{
			crc_bytes = 21;
		}
		else
		{
			crc_bytes = 17;
		}

		mCrcValue = 0;
		for (U32 i = 0; i < crc_bytes; i++)
		{
			mCrcValue <<= 1;
			BitState bit;

			/* Rule for flagging fixed stuff bits in CAN-FD frames */
			/* Every 4th bit is a fixed stuffing bit in the CRC field. */
			if ((i > 0) && ((i % 4) == 0))
			{
				done = UnstuffFixedStuffBit(fsb2, last_sample);
				if (done == true)
					return;
			}

			if (i == 0)
				done = UnstuffRawFrameBit(bit, first_sample);
			else
				done = UnstuffRawFrameBit(bit, last_sample);

			if (done == true)
				return;

			mCrcFieldWithoutDelimiter.push_back(bit);

			if (bit == mSettings->Recessive())
				mCrcValue |= 1;
		}

		frame.mStartingSampleInclusive = first_sample;
		frame.mEndingSampleInclusive = last_sample;
		frame.mType = CrcField;
		frame.mData1 = mCrcValue;
		mResults->AddFrame(frame);

	}
	else
	{
		/* CRC section for Standard CAN */		
		/* Standard 11 or 29-bit CAN - always 15 bits and normal stuffing behaviour */
		crc_bytes = 15;

		mCrcValue = 0;
		for (U32 i = 0; i < crc_bytes; i++)
		{
			mCrcValue <<= 1;
			BitState bit;

			if (i == 0)
				done = UnstuffRawFrameBit(bit, first_sample);
			else
				done = UnstuffRawFrameBit(bit, last_sample);

			if (done == true)
				return;

			mCrcFieldWithoutDelimiter.push_back(bit);

			if (bit == mSettings->Recessive())
				mCrcValue |= 1;
		}

		frame.mStartingSampleInclusive = first_sample;
		frame.mEndingSampleInclusive = last_sample;
		frame.mType = CrcField;
		frame.mData1 = mCrcValue;
		mResults->AddFrame(frame);
	}
	
	/* Trailer section is common to all formats */

	done = UnstuffRawFrameBit(mCrcDelimiter, first_sample);

	if (done == true)
		return;

	if ((frametype == FDIdentifierEx) || (frametype == FDIdentifier))
	{
		/* Add one extra bit time at this point - given examples accessible to code author have 2 bits of delimiter */
		/* Protocol specification allows for one or two bits at this point */
		/* Potentially make this optional to the user via menu. */
		BitState crc_delim2;
		done = UnstuffRawFrameBit(crc_delim2, last_sample);
		if (done == true)
			return;
	}

	/* If using flexible data rate, fast bit timing should finish here */
    mBaudSwitch = false;

	BitState ackslot;
	done = GetFixedFormFrameBit(ackslot, first_sample);

	mAckField.push_back(ackslot);
	if (ackslot == mSettings->Dominant())
		mAck = true;
	else
		mAck = false;

	BitState ackdelim;
	done = GetFixedFormFrameBit(ackdelim, last_sample);

	if (done == true)
		return;

	mAckField.push_back(ackdelim);

	frame.mStartingSampleInclusive = first_sample;
	frame.mEndingSampleInclusive = last_sample;
	frame.mType = AckField;
	frame.mData1 = mAck;
	mResults->AddFrame(frame);


	mResults->CommitPacketAndStartNewPacket();
}

bool CAN_FDAnalyzer::GetFixedFormFrameBit(BitState& result, U64& sample)
{
	if (mNumRawBits <= mRawFrameIndex)
		return true;

	result = mRawBitResults[mRawFrameIndex];
	sample = mStartOfFrame + mSampleOffsets[mRawFrameIndex];
	mCanMarkers.push_back(CanMarker(sample, Standard));

	if (mBaudSwitch == true)
	{
		mRawFrameIndex++;
	}
	else
	{
		mRawFrameIndex += mBitRateFactor;
	}

	return false;
}

bool CAN_FDAnalyzer::UnstuffFixedStuffBit(BitState& result, U64& sample, bool reset)
{
	/* Some bits in CAN-FD are known stuffing bits. We mark these as stuffed bits */
	/* Note that this routine does not provide a result which should be used as part */
	/* of a data field in the message, but it does then increment the FrameIndex to */
	/* point at the next bit which will be part of the message */
	
	/* Initial check to ensure we aren't beyond the end of the raw frame */
	if (mRawFrameIndex >= mNumRawBits)
		return true;

	/* Get this sample */
	sample = mStartOfFrame + mSampleOffsets[mRawFrameIndex];
	result = mRawBitResults[mRawFrameIndex];

	/* Fixed stuffing bit used by CAN-FD protocol */
	if (result == mSettings->Recessive())
	{
		mDominantCount = 0;
		mRecessiveCount++;
	}
	else
	{
		mRecessiveCount = 0;
		mDominantCount++;
	}
	mCanMarkers.push_back(CanMarker(sample, BitStuff));
	/* Next bit is always one data bit time from here */
	mRawFrameIndex++;

	return false;
}

bool CAN_FDAnalyzer::UnstuffRawFrameBit(BitState& result, U64& sample, bool reset)
{
	/* This routine is the one normally called by the frame analysis function to unpack bits to be included */
	/* It also acts as the place which detects if the CAN or CAN-FD protocol has added a stuffing bit into the */
	/* protocol. This is important, since the code which called this routine is expecting the bits to be */
	/* according to a specified protocol. Depending on the values of real data in this packet, the protocol rules */
	/* may add a stuffing bit or not into the serial data. This routine must identify if bit stuffing has taken place */
	/* mark the bit as one which is an inserted bit, and then go on to process the next bit in the frame which will be */
	/* the bit that was expected originally. This routine, therefore, unless set to reset the frame - *must* return */
	/* the result of the expected bit, and then finish by incrementing the FramINdex by an appropriate value. */
	/* Returns true if FrameIndex now points past the end of the acquired data, otherwise returns false. */
	
	if (reset == true)
	{
		mRecessiveCount = 0;
		mDominantCount = 0;
		mCanMarkers.clear();
		mRawFrameIndex = 1; /* get away from the leading sample edge just a little */
	}

	/* Initial check to ensure we aren't beyond the end of the raw frame */
	if (mRawFrameIndex >= mNumRawBits)
		return true;

	/* Get the number of this sample */
	sample = mStartOfFrame + mSampleOffsets[mRawFrameIndex];

	/* Check for a dominant bit stuffing bit */
	if (mRecessiveCount == 5)
	{
		mRecessiveCount = 0;
		mDominantCount = 1; //this bit is DOMINANT, and counts towards the next bit stuff
		mCanMarkers.push_back(CanMarker(sample, BitStuff));

		/* Point at next bit */
		if (mBaudSwitch == true)
		{
			mRawFrameIndex++;
		}
		else
		{
			mRawFrameIndex += mBitRateFactor;
		}
	}

	/* Check for a recessive bit stuffing bit */
	if (mDominantCount == 5)
	{
		mDominantCount = 0;
		mRecessiveCount = 1; //this bit is RECESSIVE, and counts towards the next bit stuff
		mCanMarkers.push_back(CanMarker(sample, BitStuff));

		/* Point at next bit */
		if (mBaudSwitch == true)
		{
			mRawFrameIndex++;
		}
		else
		{
			mRawFrameIndex += mBitRateFactor;
		}
	}

	/* Check to ensure we aren't beyond the end of the raw frame */
	if (mRawFrameIndex >= mNumRawBits)
		return true;

	/* This bit contributes to message */
	sample = mStartOfFrame + mSampleOffsets[mRawFrameIndex];
	result = mRawBitResults[mRawFrameIndex];

	if (result == mSettings->Recessive())
	{
		mRecessiveCount++;
		mDominantCount = 0;
	}
	else
	{
		mDominantCount++;
		mRecessiveCount = 0;
	}

	/* Add marker */
	mCanMarkers.push_back(CanMarker(sample, Standard));

	/* Get next sample - may need adjustment if a baud switch is being made */

	if ((mCheckingBRS == true) && (result == mSettings->Recessive()))
	{
		mBaudSwitch = true;
		/* Must go to next edge now */
		if (mBitRateFactor < 2)
		{
			/* Catch for when header and data rates are the same with the BRS bit set */
			mRawFrameIndex++;
		}
		else
		{
			/* Puts next bit to process as the ESI bit */
			mRawFrameIndex += (U32)(mBitRateFactor - 2);
		}
	}
	else
	{
		if (mBaudSwitch == true)
		{
			mRawFrameIndex++;
		}
		else
		{
			mRawFrameIndex += mBitRateFactor;
		}
	}

	return false;
}



bool CAN_FDAnalyzer::NeedsRerun()
{
	return false;
}

U32 CAN_FDAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
	if( mSimulationInitilized == false )
	{
		mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
		mSimulationInitilized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 CAN_FDAnalyzer::GetMinimumSampleRateHz()
{
	/* Ensure we have at least 4 samples per bit at the higher bit rate of the header and data */
	
	if (mSettings->mBitRateData >= mSettings->mBitRateHdr)
	{
		return mSettings->mBitRateData * 4;
	}
	else
	{
		return mSettings->mBitRateHdr * 4;
	}
}

const char* CAN_FDAnalyzer::GetAnalyzerName() const
{
	return "CAN_FD";
}

const char* GetAnalyzerName()
{
	return "CAN_FD";
}

Analyzer* CreateAnalyzer()
{
	return new CAN_FDAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
	delete analyzer;
}