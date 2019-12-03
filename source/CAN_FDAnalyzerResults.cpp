#include "CAN_FDAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CAN_FDAnalyzer.h"
#include "CAN_FDAnalyzerSettings.h"
#include <iostream>
#include <sstream>

CAN_FDAnalyzerResults::CAN_FDAnalyzerResults( CAN_FDAnalyzer* analyzer, CAN_FDAnalyzerSettings* settings )
:	AnalyzerResults(),
	mSettings( settings ),
	mAnalyzer( analyzer )
{
}

CAN_FDAnalyzerResults::~CAN_FDAnalyzerResults()
{
}

void CAN_FDAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
	//we only need to pay attention to 'channel' if we're making bubbles for more than one channel (as set by AddChannelBubblesWillAppearOn)
	ClearResultStrings();
	Frame frame = GetFrame(frame_index);

	switch (frame.mType)
	{
	case IdentifierField:
	case IdentifierFieldEx:
	{
		char number_str[128];

		if (frame.mType == IdentifierField)
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 12, number_str, 128);
		else
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str, 128);

		std::stringstream ss;

		AddResultString("Id");

		ss << "Id: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		ss << "Identifier: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		if (frame.HasFlag(REMOTE_FRAME) == false)
		{
			if (frame.mType == IdentifierField)
				ss << "11-bit CAN Identifier: " << number_str;
			else
				ss << "29-bit CAN Identifier: " << number_str;
		}
		else
		{
			if (frame.mType == IdentifierField)
				ss << "11-bit CAN Identifier: " << number_str << " (RTR)";
			else
				ss << "29-bit CAN Identifier: " << number_str << " (RTR)";
		}

		AddResultString(ss.str().c_str());
	}
	break;

	case FDIdentifier:
	case FDIdentifierEx:
	{
		char number_str[128];

		if (frame.mType == FDIdentifier)
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 12, number_str, 128);
		else
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str, 128);

		std::stringstream ss;

		AddResultString("Id");

		ss << "FD-Id: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		ss << "FD-Identifier: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		if (frame.HasFlag(REMOTE_FRAME) == false)
		{
			if (frame.mType == FDIdentifier)
				ss << "11-bit CAN-FD Identifier: " << number_str;
			else
				ss << "29-bit CAN-FD Identifier: " << number_str;
		}
		else
		{
			/* Note - remote frames are not used according to the spec */
			if (frame.mType == FDIdentifier)
				ss << "11-bit CAN-FD Identifier: " << number_str << " (RTR)";
			else
				ss << "29-bit CAN-FD Identifier: " << number_str << " (RTR)";
		}

		AddResultString(ss.str().c_str());
	}
	break;

	case ControlField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 4, number_str, 128);

		std::stringstream ss;
		AddResultString("Ctrl");

		ss << "Ctrl: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		ss << "Control Field: " << number_str;
		AddResultString(ss.str().c_str());

		ss << " bytes";
		AddResultString(ss.str().c_str());
	}
	break;

	case DataField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);

		AddResultString(number_str);

		std::stringstream ss;
		ss << "Data: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		ss << "Data Field Byte: " << number_str;
		AddResultString(ss.str().c_str());
	}
	break;

	case CrcField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 15, number_str, 128);

		AddResultString("CRC");

		std::stringstream ss;
		ss << "CRC: " << number_str;
		AddResultString(ss.str().c_str());
		ss.str("");

		ss << "CRC value: " << number_str;
		AddResultString(ss.str().c_str());
	}
	break;

	case AckField:
	{
		if (bool(frame.mData1) == true)
			AddResultString("ACK");
		else
			AddResultString("NAK");
	}
	break;

	case CanError:
	{
		AddResultString("E");
		AddResultString("Error");
	}
	break;

	}
}

void CAN_FDAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
	//export_type_user_id is only important if we have more than one export type.
	std::stringstream ss;
	void* f = AnalyzerHelpers::StartFile(file);

	U64 trigger_sample = mAnalyzer->GetTriggerSample();
	U32 sample_rate = mAnalyzer->GetSampleRate();

	ss << "Time [s],Packet,Type,Identifier,Control,Data,CRC,ACK" << std::endl;
	U64 num_frames = GetNumFrames();
	U64 num_packets = GetNumPackets();
	for (U32 i = 0; i < num_packets; i++)
	{
		if (i != 0)
		{
			//below, we "continue" the loop rather than run to the end.  So we need to save to the file here.
			ss << std::endl;

			AnalyzerHelpers::AppendToFile((U8*)ss.str().c_str(), (U32)ss.str().length(), f);
			ss.str(std::string());


			if (UpdateExportProgressAndCheckForCancel(i, num_packets) == true)
			{
				AnalyzerHelpers::EndFile(f);
				return;
			}
		}


		U64 first_frame_id;
		U64 last_frame_id;
		GetFramesContainedInPacket(i, &first_frame_id, &last_frame_id);
		Frame frame = GetFrame(first_frame_id);

		//static void GetTimeString( U64 sample, U64 trigger_sample, U32 sample_rate_hz, char* result_string, U32 result_string_max_length );
		char time_str[128];
		AnalyzerHelpers::GetTimeString(frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128);

		char packet_str[128];
		AnalyzerHelpers::GetNumberString(i, Decimal, 0, packet_str, 128);

		if (frame.HasFlag(REMOTE_FRAME) == false)
			ss << time_str << "," << packet_str << ",DATA";
		else
			ss << time_str << "," << packet_str << ",REMOTE";

		U64 frame_id = first_frame_id;

		frame = GetFrame(frame_id);

		char number_str[128];

		if ((frame.mType == IdentifierField) || (frame.mType == FDIdentifier))
		{
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 12, number_str, 128);
			ss << "," << number_str;
			++frame_id;
		}
		else if ((frame.mType == IdentifierFieldEx) || (frame.mType == FDIdentifierEx))
		{
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str, 128);
			ss << "," << number_str;
			++frame_id;
		}
		else
		{
			ss << ",";
		}

		if (frame_id > last_frame_id)
			continue;

		frame = GetFrame(frame_id);
		if (frame.mType == ControlField)
		{
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 4, number_str, 128);
			ss << "," << number_str;
			++frame_id;
		}
		else
		{
			ss << ",";
		}
		ss << ",";
		if (frame_id > last_frame_id)
			continue;

		for (; ; )
		{
			frame = GetFrame(frame_id);
			if (frame.mType != DataField)
				break;

			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);
			ss << number_str;
			if (frame_id == last_frame_id)
				break;

			++frame_id;
			if (GetFrame(frame_id).mType == DataField)
				ss << " ";
		}

		if (frame_id > last_frame_id)
			continue;

		frame = GetFrame(frame_id);
		if (frame.mType == CrcField)
		{
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 15, number_str, 128);
			ss << "," << number_str;
			++frame_id;
		}
		else
		{
			ss << ",";
		}
		if (frame_id > last_frame_id)
			continue;

		frame = GetFrame(frame_id);
		if (frame.mType == AckField)
		{
			if (bool(frame.mData1) == true)
				ss << "," << "ACK";
			else
				ss << "," << "NAK";

			++frame_id;
		}
		else
		{
			ss << ",";
		}
	}

	UpdateExportProgressAndCheckForCancel(num_frames, num_frames);
	AnalyzerHelpers::EndFile(f);
}

void CAN_FDAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
	ClearTabularText();

	Frame frame = GetFrame(frame_index);

	switch (frame.mType)
	{
	case IdentifierField:
	case IdentifierFieldEx:
	{
		char number_str[128];

		if (frame.mType == IdentifierField)
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 12, number_str, 128);
		else
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str, 128);

		std::stringstream ss;


		if (frame.HasFlag(REMOTE_FRAME) == false)
		{
			if (frame.mType == IdentifierField)
				ss << "Standard CAN Identifier: " << number_str;
			else
				ss << "Extended CAN Identifier: " << number_str;
		}
		else
		{
			if (frame.mType == IdentifierField)
				ss << "Standard CAN Identifier: " << number_str << " (RTR)";
			else
				ss << "Extended CAN Identifier: " << number_str << " (RTR)";
		}

		AddTabularText(ss.str().c_str());
	}
	break;

	case FDIdentifier:
	case FDIdentifierEx:
	{
		char number_str[128];

		if (frame.mType == FDIdentifier)
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 12, number_str, 128);
		else
			AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 32, number_str, 128);

		std::stringstream ss;


		if (frame.HasFlag(REMOTE_FRAME) == false)
		{
			if (frame.mType == FDIdentifier)
				ss << "11-bit FD-CAN Identifier: " << number_str;
			else
				ss << "29-bit FD-CAN Identifier: " << number_str;
		}
		else
		{
			if (frame.mType == FDIdentifier)
				ss << "11-bit FD-CAN Identifier: " << number_str << " (RTR)";
			else
				ss << "29-bit FD-CAN Identifier: " << number_str << " (RTR)";
		}

		AddTabularText(ss.str().c_str());
	}
	break;

	case ControlField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 4, number_str, 128);

		std::stringstream ss;

		ss << "Control Field: " << number_str;
		ss << " bytes";
		AddTabularText(ss.str().c_str());

	}
	break;
	case DataField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 8, number_str, 128);

		std::stringstream ss;

		ss << "Data Field Byte: " << number_str;
		AddTabularText(ss.str().c_str());
	}
	break;
	case CrcField:
	{
		char number_str[128];
		AnalyzerHelpers::GetNumberString(frame.mData1, display_base, 15, number_str, 128);

		std::stringstream ss;

		ss << "CRC value: " << number_str;
		AddTabularText(ss.str().c_str());
	}
	break;
	case AckField:
	{
		if (bool(frame.mData1) == true)
			AddTabularText("ACK");
		else
			AddTabularText("NAK");
	}
	break;
	case CanError:
	{
		AddTabularText("Error");
	}
	break;
	}
}

void CAN_FDAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
	ClearResultStrings();
	AddResultString("not supported");

}

void CAN_FDAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
	ClearResultStrings();
	AddResultString("not supported");
}