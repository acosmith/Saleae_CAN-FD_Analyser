#ifndef CAN_FD_ANALYZER_SETTINGS
#define CAN_FD_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class CAN_FDAnalyzerSettings : public AnalyzerSettings
{
public:
	CAN_FDAnalyzerSettings();
	virtual ~CAN_FDAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings( const char* settings );
	virtual const char* SaveSettings();

	
	Channel mInputChannel;
	U32 mBitRateHdr;
	U32 mBitRateData;
	bool mInverted;

	BitState Recessive();
	BitState Dominant();


protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >	mInputChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceInteger >	mBitRateHdrInterface;
	std::auto_ptr< AnalyzerSettingInterfaceInteger >	mBitRateDataInterface;
	std::auto_ptr< AnalyzerSettingInterfaceBool > mInvertedInterface;
};

#endif //CAN_FD_ANALYZER_SETTINGS
