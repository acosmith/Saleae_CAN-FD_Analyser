#ifndef CAN_FD_SIMULATION_DATA_GENERATOR
#define CAN_FD_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <string>
#include <AnalyzerHelpers.h>

class CAN_FDAnalyzerSettings;

class CAN_FDSimulationDataGenerator
{
public:
	CAN_FDSimulationDataGenerator();
	~CAN_FDSimulationDataGenerator();

	void Initialize( U32 simulation_sample_rate, CAN_FDAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	CAN_FDAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;

protected:  //functions

	U16 ComputeCrc(std::vector<BitState>& bits, U32 num_bits);
	void AddCrc();
	void CreateDataOrRemoteFrame(U32 identifier, bool use_extended_frame_format, bool remote_frame, std::vector<U8>& data, bool get_ack_in_response);
	void WriteFrame(bool error = false);

protected:  //variables

	ClockGenerator mClockGeneratorHdr;
	ClockGenerator mClockGeneratorData;
	SimulationChannelDescriptor mCanFDSimulationData;  //if we had more than one channel to simulate, they would need to be in an array

	U8 mValue;

	std::vector<BitState> mFakeStartOfFrameField;
	std::vector<BitState> mFakeArbitrationField;
	std::vector<BitState> mFakeControlField;
	std::vector<BitState> mFakeDataField;
	std::vector<BitState> mFakeCrcFieldWithoutDelimiter;
	std::vector<BitState> mFakeAckField;
	std::vector<BitState> mFakeEndOfFrame;
	std::vector<BitState> mFakeStuffedBits;
	std::vector<BitState> mFakeFixedFormBits;

};
#endif //CAN_FD_SIMULATION_DATA_GENERATOR