#ifndef CAN_FD_ANALYZER_H
#define CAN_FD_ANALYZER_H

#include <Analyzer.h>
#include "CAN_FDAnalyzerResults.h"
#include "CAN_FDSimulationDataGenerator.h"

enum CanBitType { Standard, BitStuff };

class CanMarker
{
public:
	CanMarker(U64 sample, enum CanBitType type)
	{
		mSample = sample;
		mType = type;
	}

	U64 mSample;
	enum CanBitType mType;
};

class CAN_FDAnalyzerSettings;

class ANALYZER_EXPORT CAN_FDAnalyzer : public Analyzer2
{
public:
	CAN_FDAnalyzer();
	virtual ~CAN_FDAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();


protected: //analysis functions
	void WaitFor7RecessiveBits();
	void InitSampleOffsets();
	void GetRawFrame();
	void AnalyzeRawFrame();
	bool UnstuffFixedStuffBit(BitState& result, U64& sample, bool reset = false);
	bool UnstuffRawFrameBit(BitState& result, U64& sample, bool reset = false);
	bool GetFixedFormFrameBit(BitState& result, U64& sample);

protected: //vars
	std::auto_ptr< CAN_FDAnalyzerSettings > mSettings;
	std::auto_ptr< CAN_FDAnalyzerResults > mResults;
	AnalyzerChannelData* mCAN_FD;
	U32 mSampleRateHz;

	CAN_FDSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;


protected: //analysis vars:

	U32 mBitRateFactor;
	bool mCheckingBRS;
	bool mBaudSwitch;

	U32 mNumSamplesIn7Bits;
	U32 mRecessiveCount;
	U32 mDominantCount;
	U32 mRawFrameIndex;
	U64 mStartOfFrame;
	U32 mIdentifier;
	U32 mCrcValue;
	bool mAck;

	std::vector<U32> mSampleOffsets;
	std::vector<BitState> mRawBitResults;
	std::vector<BitState> mBitResults;
	std::vector<CanMarker> mCanMarkers;
	std::vector<BitState> mArbitrationField;

	bool mStandardCan;
	bool mStandardCanFD;
	bool mRemoteFrame;
	U32 mNumDataBytes;

	std::vector<BitState> mControlField;
	std::vector<BitState> mDataField;
	std::vector<BitState> mCrcFieldWithoutDelimiter;
	BitState mCrcDelimiter;
	std::vector<BitState> mAckField;
	std::vector<BitState> mEndOfFrame;

	U32 mNumRawBits;
	bool mCanError;
	U64 mErrorStartingSample;
	U64 mErrorEndingSample;
	
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //CAN_FD_ANALYZER_H
