#ifndef CAN_FD_ANALYZER_RESULTS
#define CAN_FD_ANALYZER_RESULTS

#include <AnalyzerResults.h>

enum CanFrameType { IdentifierField, IdentifierFieldEx, FDIdentifier, FDIdentifierEx, ControlField, DataField, CrcField, AckField, CanError };
#define REMOTE_FRAME ( 1 << 0 )

class CAN_FDAnalyzer;
class CAN_FDAnalyzerSettings;

class CAN_FDAnalyzerResults : public AnalyzerResults
{
public:
	CAN_FDAnalyzerResults( CAN_FDAnalyzer* analyzer, CAN_FDAnalyzerSettings* settings );
	virtual ~CAN_FDAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions

protected:  //vars
	CAN_FDAnalyzerSettings* mSettings;
	CAN_FDAnalyzer* mAnalyzer;
};

#endif //CAN_FD_ANALYZER_RESULTS
