#include "CAN_FDAnalyzerSettings.h"
#include <AnalyzerHelpers.h>


CAN_FDAnalyzerSettings::CAN_FDAnalyzerSettings()
:	mInputChannel( UNDEFINED_CHANNEL ),
    mBitRateHdr ( 1000000 ),
	mBitRateData ( 1000000 ),
	mInverted (false)
{
	mInputChannelInterface.reset( new AnalyzerSettingInterfaceChannel() );
	mInputChannelInterface->SetTitleAndTooltip( "CAN-FD", "Controller Area Network (Flexible Data Rate) - Input" );
	mInputChannelInterface->SetChannel( mInputChannel );

	mBitRateHdrInterface.reset( new AnalyzerSettingInterfaceInteger() );
	mBitRateHdrInterface->SetTitleAndTooltip( "Header Bit Rate (Bits/S)",  "Specify the header bit rate in bits per second." );
	mBitRateHdrInterface->SetMax( 8000000 );
	mBitRateHdrInterface->SetMin( 1 );
	mBitRateHdrInterface->SetInteger(mBitRateHdr);

	mBitRateDataInterface.reset(new AnalyzerSettingInterfaceInteger());
	mBitRateDataInterface->SetTitleAndTooltip("Data Bit Rate (Bits/S)", "Specify the data bit rate in bits per second.");
	mBitRateDataInterface->SetMax(8000000);
	mBitRateDataInterface->SetMin(1);
	mBitRateDataInterface->SetInteger(mBitRateData);

	mInvertedInterface.reset(new AnalyzerSettingInterfaceBool());
	mInvertedInterface->SetTitleAndTooltip("Inverted (CAN High)", "Use this option when recording CAN High directly");
	mInvertedInterface->SetValue(mInverted);

	AddInterface(mInputChannelInterface.get());
	AddInterface(mBitRateHdrInterface.get());
	AddInterface(mBitRateDataInterface.get());
	AddInterface(mInvertedInterface.get());

	AddExportOption( 0, "Export as text/csv file" );
	AddExportExtension( 0, "text", "txt" );
	AddExportExtension( 0, "csv", "csv" );

	ClearChannels();
	AddChannel( mInputChannel, "Serial", false );
}

CAN_FDAnalyzerSettings::~CAN_FDAnalyzerSettings()
{
}

bool CAN_FDAnalyzerSettings::SetSettingsFromInterfaces()
{
	Channel can_chan;
	U32 hdrrate;
	U32 datarate;

	can_chan = mInputChannelInterface->GetChannel();
	hdrrate = mBitRateHdrInterface->GetInteger();
	datarate = mBitRateDataInterface->GetInteger();

	if (can_chan == UNDEFINED_CHANNEL)
	{
		SetErrorText("Please select a valid input channel for the CAN-FD interface.");
		return false;
	}

	if (datarate < hdrrate)
	{
		SetErrorText("Data rate must be greater than or equal to header rate.");
		return false;
	}

	if ((datarate % hdrrate) != 0)
	{
		SetErrorText("Data rate must be an integer multiple of header rate.");
		return false;
	}
	
	mInputChannel = can_chan;
	mBitRateHdr = hdrrate;
	mBitRateData = datarate;
	mInverted = mInvertedInterface->GetValue();

	ClearChannels();
	AddChannel( mInputChannel, "CAN_FD", true );

	return true;
}

void CAN_FDAnalyzerSettings::UpdateInterfacesFromSettings()
{
	mInputChannelInterface->SetChannel( mInputChannel );
	mBitRateHdrInterface->SetInteger( mBitRateHdr );
	mBitRateDataInterface->SetInteger( mBitRateData );
	mInvertedInterface->SetValue( mInverted );
}

void CAN_FDAnalyzerSettings::LoadSettings( const char* settings )
{
	SimpleArchive text_archive;
	text_archive.SetString( settings );

	const char* name_string;

	text_archive >> &name_string;

	if (strcmp(name_string, "CAN-FD Analyser") != 0)
		AnalyzerHelpers::Assert("CAN-FD Analyser: Settings string does not belong to...");

	text_archive >> mInputChannel;
	text_archive >> mBitRateHdr;
	text_archive >> mBitRateData;
	text_archive >> mInverted;

	ClearChannels();
	AddChannel( mInputChannel, "CAN_FD", true );

	UpdateInterfacesFromSettings();
}

const char* CAN_FDAnalyzerSettings::SaveSettings()
{
	SimpleArchive text_archive;

	text_archive << "CAN-FD Analyser";
	text_archive << mInputChannel;
	text_archive << mBitRateHdr;
	text_archive << mBitRateData;
	text_archive << mInverted;

	return SetReturnString( text_archive.GetString() );
}

BitState CAN_FDAnalyzerSettings::Recessive()
{
	if (mInverted)
		return BIT_LOW;
	return BIT_HIGH;
}
BitState CAN_FDAnalyzerSettings::Dominant()
{
	if (mInverted)
		return BIT_HIGH;
	return BIT_LOW;
}