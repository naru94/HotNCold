#include <gtk/gtk.h>
#include <glib/gprintf.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <glib.h>
#include <pthread.h>
#include <string.h>
#include <gdk/gdk.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/reboot.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>       /* for struct timeval {} */
#include <stdint.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <ifaddrs.h> 
#include <linux/rtnetlink.h>
#include <errno.h>

#define ROLLER_UP    0x10
#define ROLLER_DWN   0x20 
#define ROLLER_HALT  0x00

char Hardware_Version[] = "1.0";  
char Firmware_Version[] = "1.0";

char Device[2][25]={
	{"Twin Rubber"},
    {"Steel Rubber"}
};

char u8AlarmStr[7][25]={
	{"No Alarm"},
	{"Roller Up Temp High"},
	{"Roller Down Temp High"},
	{"IR Sensor Error-UP"},
	{"IR Sensor Error-Down"},
	{"KTC Sensor Error-Up"},
	{"KTC Sensor Error-Down"}
};

// enum
// {
// NO_ALARM = 0x00,
// ROLLER_UP_TEMP_HIGH,
// ROLLER_DOWN_TEMP_HIGH,
// AMBIENT_UP_TEMP_HIGH,
// AMBIENT_DOWN_TEMP_HIGH,
// IR_SENSOR_UP_ERROR,
// IR_SENSOR_DOWN_ERROR,   
// KTC_SENSOR_UP_ERROR,//K type Thermocouple means KTC
// KTC_SENSOR_DOWN_ERROR,
// }SYSTEM_ALARMS;

void *Data_Background_thread()
{
	
}

int PlusState = 0;
int MinusState = 0;

int TempPlusState1 = 0;
int TempMinusState1 = 0;
int CtrlOffsetPlusState1 = 0;
int CtrlOffsetMinusState1 = 0;
int CtrlOutputPlusState1 = 0;
int CtrlOutputMinusState1 = 0;
int TempOffsetPlusState1 = 0;
int TempOffsetMinusState1 = 0;

int TempPlusState2 = 0;
int TempMinusState2 = 0;
int CtrlOffsetPlusState2 = 0;
int CtrlOffsetMinusState2 = 0;
int CtrlOutputPlusState2 = 0;
int CtrlOutputMinusState2 = 0;
int TempOffsetPlusState2 = 0;
int TempOffsetMinusState2 = 0;

int SettingState = 0;

int16_t Heater1 = 0;
int16_t Heater2 = 0;
int16_t Laser = 0;
int16_t LaserStatus = 0;
int16_t MotorState = 0;

int16_t Current_Up_Temperature;
int16_t Current_Down_Temperature;

int16_t Set_Temp_Value1 = 30; 
int16_t Set_Temp_Value2 = 30;
int16_t Temperature1 = 30;
int16_t Temperature2 = 30;

int16_t Speed = 10; 
int16_t MotorDirection = 1;		// 1 - Forward  0 - Backward
int16_t RollerDirection = ROLLER_HALT;
gboolean RollerDownState, RollerUpState;

int16_t Set_CtrlOffset_Value1 = 0;
int16_t Set_CtrlOffset_Value2 = 20;
int16_t ControlOffset1 = 0;
int16_t ControlOffset2 = 20;

int16_t Set_CtrlOutput_Value1 = 10;
int16_t Set_CtrlOutput_Value2 = 100;
int16_t ControlOutput1 = 10;
int16_t ControlOutput2 = 100;

int16_t Set_TempOffset_Value1 = -25;
int16_t Set_TempOffset_Value2 = 25;
int16_t TemperatureOffset1 = -25;
int16_t TemperatureOffset2 = 25;

int16_t SystemWarning=1;

int16_t AmbientTemperature1 = 25;
int16_t AmbientTemperature2 = 25;

int16_t DeviceType=1;

int NumState1=0;
int NumState2=0;
int NumState3=0;
int NumState4=0;
int NumState5=0;
int NumState6=0;
int NumState7=0;
int NumState8=0;
int NumState9=0;
int NumState0=0;

char password_in[4];
int passwordlen=0;

typedef struct MODBUS_REGISTERS{
    int16_t Set_Up_Temperature;			//1
    int16_t Set_Down_Temperature;		//2
    int16_t Motor_Speed_Percentage;		//3
    int16_t Heater_ON_OFF_Up_Sensor;	//4
    int16_t Heater_ON_OFF_Down_Sensor;	//5
    int16_t Rod_Up_Down;				//6
    int16_t Motor_Direction_Control;	//7
    int16_t Motor_Start_Stop;			//8
    int16_t Sensor_OffSet_1;			//9 ---Reading Calibration + - Offset point of Roller_1 Down side 
    int16_t Sensor_OffSet_2;			//10---Reading Calibration + - Offset point of Roller_2 Up side 
    int16_t Control_OffSet_1;			//11 0-200 ie 0-20 degree Celcius it is pre braking point from where the
    int16_t Control_OffSet_2;			//12 heat reduces to a certain percentage.
    int16_t Control_Output_Up_Sensor;	//13 This value is in percentage
    int16_t Control_Output_Down_Sensor;	//14  10% to 100%
    int16_t Laser_Control_Reg;			//15
    int16_t Rod_Rotation_Pulses_Reset;	//16----Area Reset in Old Document
    int16_t Current_Up_Temperature;		//17
    int16_t Ambient_Up_Temperature;		//18
    int16_t Current_Down_Temperature;	//19
    int16_t Ambient_Down_Temperature;	//20
    int16_t Rod_Rotation_Pulses;		//21
    int16_t Emergency_Switch_Input;		//22
    int16_t NTC_Sensor_1_Status;		//23
    int16_t NTC_Sensor_2_Status;		//24
    int16_t System_Alarm;				//25
}MODBUS_REGISTERS;

MODBUS_REGISTERS STDregisters;

typedef struct
{
	GtkWidget *TimeLabel1, *DateLabel1;
	GtkWidget *RollTemp1, *RollTemp2;
	GtkWidget *SetTempDisp1, *SetTempDisp2;
	GtkWidget *SpeedPlusDown, *SpeedPlusUp, *SpeedMinusDown, *SpeedMinusUp, *SettingButtonUp, *SettingButtonDown;
	GtkWidget *SpeedEntry;
	GtkWidget *SetCtrlOffsetDisp1, *SetCtrlOffsetDisp2;
	GtkWidget *SetCtrlOutputDisp1, *SetCtrlOutputDisp2;
	GtkWidget *SetTempOffsetDisp1, *SetTempOffsetDisp2;
	GtkWidget *TempMinusDown1, *TempMinusUp1, *TempPlusDown1, *TempPlusUp1, *TempEntry1;
	GtkWidget *TempMinusDown2, *TempMinusUp2, *TempPlusDown2, *TempPlusUp2, *TempEntry2;
	GtkWidget *HandSafetyOFF;
	
	GtkWidget *CtrlOffsetMinusDown1, *CtrlOffsetMinusUp1, *CtrlOffsetPlusDown1, *CtrlOffsetPlusUp1, *CtrlOffsetEntry1;
	GtkWidget *CtrlOffsetMinusDown2, *CtrlOffsetMinusUp2, *CtrlOffsetPlusDown2, *CtrlOffsetPlusUp2, *CtrlOffsetEntry2;
	GtkWidget *CtrlOutputMinusDown1, *CtrlOutputMinusUp1, *CtrlOutputPlusDown1, *CtrlOutputPlusUp1, *CtrlOutputEntry1;
	GtkWidget *CtrlOutputMinusDown2, *CtrlOutputMinusUp2, *CtrlOutputPlusDown2, *CtrlOutputPlusUp2, *CtrlOutputEntry2;
	GtkWidget *TempOffsetMinusDown1, *TempOffsetMinusUp1, *TempOffsetPlusDown1, *TempOffsetPlusUp1, *TempOffsetEntry1;
	GtkWidget *TempOffsetMinusDown2, *TempOffsetMinusUp2, *TempOffsetPlusDown2, *TempOffsetPlusUp2, *TempOffsetEntry2;
	
	GtkWidget *SystemWarningDisp, *AmbientTempDisp1, *AmbientTempDisp2, *DeviceTypeDisp;
	
	GtkWidget *Dwn1, *Dwn2, *Dwn3, *Dwn4, *Dwn5, *Dwn6, *Dwn7, *Dwn8, *Dwn9, *Dwn0;
	GtkWidget *Up1, *Up2, *Up3, *Up4, *Up5, *Up6, *Up7, *Up8, *Up9, *Up0;
	GtkWidget *PinEntry1, *PinEntry2, *PinEntry3, *PinEntry4;
} app_widgets;

typedef struct 
{
	app_widgets *Window;
	int Status;
} Thread_Arguments;

GtkBuilder *builder;
GtkWidget *HomeWindow, *HomeWlayout;

GtkWidget *Background1, *TimeIcon1, *XLJetHeader1, *DateIcon1;
GtkWidget *HeaterToggle1, *HeaterToggle2, *MinusButton, *PlusButton, *SettingsButton, *MotorRButton, *MotorFButton;
GtkToggleButton *RollerDownButton, *RollerUpButton, *StartButton, *HandSafetyToggle;
GtkWidget *RollerFrame1, *RollerFrame2, *RollerText1, *RollerText2, *DegreeBox1, *DegreeBox2, *SetBox1, *SetBox2;
GtkWidget *RollerHeaterON1, *RollerHeaterOFF1, *RollerHeaterON2, *RollerHeaterOFF2, *SpeedText, *SpeedDataBox;
GtkWidget *RollerUpON, *RollerUpOFF, *RollerDownON, *RollerDownOFF, *HandSafetyON, *HandSafetyDisable, *ReverseON, *ReverseOFF, *ForwardON, *ForwardOFF, *StartImage, *StopImage;

GtkWidget *Temp1;
GtkWidget *TempLayout1, *TempPlusButton1, *TempMinusButton1, *TempOKButton1, *TempCancelButton1, *PopUpBG1, *RollerUpPopup1, *SetTempPopup1, *DataBoxPopup1, *OKPopup1, *CancelPopup1;

GtkWidget *Temp2;
GtkWidget *TempLayout2, *TempPlusButton2, *TempMinusButton2, *TempOKButton2, *TempCancelButton2, *PopUpBG2, *RollerUpPopup2, *SetTempPopup2, *DataBoxPopup2, *OKPopup2, *CancelPopup2;

GtkWidget *MenuWindow;
GtkWidget *Background2, *XLJetHeader2;
GtkWidget *CalibrationBImage, *DeviceStatusBImage, *MenuExitBImage;
GtkWidget *CalibrationSButton, *DeviceSButton, *MenuExitButton;

GtkWidget *CalibrationSWindow;
GtkWidget *SetCtrlOffsetUp, *SetCtrlOffsetDwn, *SetCtrlOutputUp, *SetCtrlOutputDwn, *SetTempOffsetUp, *SetTempOffsetDwn, *CalibrationExitButton;
GtkWidget *Background3, *XLJetHeader3;
GtkWidget *ControlOffsetBox, *CtrlOffsetUpLabel, *CtrlOffsetDwnLabel, *CtrlOffsetUpImg, *CtrlOffsetDwnImg;
GtkWidget *ControlOutputBox, *CtrlOutputUpLabel, *CtrlOutputDwnLabel, *CtrlOutputUpImg, *CtrlOutputDwnImg;
GtkWidget *TempOffsetBox, *TempOffsetUpLabel, *TempOffsetDwnLabel, *TempOffsetUpImg, *TmpOffsetDwnEntry, *CalibrationExitImage;

GtkWidget *CtrlOffset1;
GtkWidget *CtrlOffsetMinusButton1, *CtrlOffsetCancelButton1, *CtrlOffsetOKButton1, *CtrlOffsetPlusButton1, *CtrlOffsetPopUpBG1, *CtrlOffsetUp1, *CtrlOffsetPop1, *CtrlOffsetBox1, *CtrlOffsetOK1, *CtrlOffsetCancel1;

GtkWidget *CtrlOffset2;
GtkWidget *CtrlOffsetMinusButton2, *CtrlOffsetCancelButton2, *CtrlOffsetOKButton2, *CtrlOffsetPlusButton2, *CtrlOffsetPopUpBG2, *CtrlOffsetUp2, *CtrlOffsetPop2, *CtrlOffsetBox2, *CtrlOffsetOK2, *CtrlOffsetCancel2;

GtkWidget *CtrlOutput1;
GtkWidget *CtrlOutputMinusButton1, *CtrlOutputCancelButton1, *CtrlOutputOKButton1, *CtrlOutputPlusButton1, *CtrlOutputPopUpBG1, *CtrlOutputUp1, *CtrlOutputPop1, *CtrlOutputBox1, *CtrlOutputOK1, *CtrlOutputCancel1;

GtkWidget *CtrlOutput2;
GtkWidget *CtrlOutputMinusButton2, *CtrlOutputCancelButton2, *CtrlOutputOKButton2, *CtrlOutputPlusButton2, *CtrlOutputPopUpBG2, *CtrlOutputUp2, *CtrlOutputPop2, *CtrlOutputBox2, *CtrlOutputOK2, *CtrlOutputCancel2;

GtkWidget *TempOffset1;
GtkWidget *TempOffsetMinusButton1, *TempOffsetCancelButton1, *TempOffsetOKButton1, *TempOffsetPlusButton1, *TempOffsetPopUpBG1, *TempOffsetUp1, *TempOffsetPop1, *TempOffsetBox1, *TempOffsetOK1, *TempOffsetCancel1;

GtkWidget *TempOffset2;
GtkWidget *TempOffsetMinusButton2, *TempOffsetCancelButton2, *TempOffsetOKButton2, *TempOffsetPlusButton2, *TempOffsetPopUpBG2, *TempOffsetUp2, *TempOffsetPop2, *TempOffsetBox2, *TempOffsetOK2, *TempOffsetCancel2;

GtkWidget *DeviceStatusWindow;
GtkWidget *DeviceStatusExitButton, *Background4, *XLJetHeader4, *SystemWarningBox, *AmbientTempBox, *AmbientTempUpLabel, *AmbientTempDownLabel, *AmbientTmpUpImg, *AmbientTmpDwnImg, *DeviceTypeBox;

GtkWidget *PasswordWindow;
GtkWidget *Num1, *Num2, *Num3, *Num4, *Num5, *Num6, *Num7, *Num8, *Num9, *Num0, *DeleteButton, *BackButton, *EnterButton;
GtkWidget *Background5, *XLJetHeader5, *EnterPin, *PinEntryBox1, *PinEntryBox2, *PinEntryBox3, *PinEntryBox4;
GtkWidget *BackImg, *EnterImg;

GtkWidget *PwdCheckWindow;
GtkWidget *PwdOKButton, *WrongPin_BG, *OkWrongBG;

GtkWidget *HomeBlurWindow;
GtkWidget *HomeBlur;

GtkWidget *CalibrationBlurWindow;
GtkWidget *CalibrationBlur;

GtkWidget *PasswordBlurWindow;
GtkWidget *PasswordBlur;

GtkWidget *EmergencyWindow;
GtkWidget *EmergencyImg;



static gboolean
on_timeout (void *user_data)
{
	Thread_Arguments  *ThreadVal = user_data;
	
	char display_time[20] = {0};
	char display_date[20] = {0};

	char str_Current1[10] = {0};
	char str_Current2[10] = {0};
	
	char str_SetTempDisp1[10] = {0};
	char str_SetTempDisp2[10] = {0};
	char str_Temp1[10] = {0};
	char str_Temp2[10] = {0};
	
	char str_Speed[10] = {0};
	
	char str_SetCtrlOffsetDisp1[10] = {0};
	char str_SetCtrlOffsetDisp2[10] = {0};
	char str_CtrlOffset1[10] = {0};
	char str_CtrlOffset2[10] = {0};
	
	char str_SetCtrlOutputDisp1[10] = {0};
	char str_SetCtrlOutputDisp2[10] = {0};
	char str_CtrlOutput1[10] = {0};
	char str_CtrlOutput2[10] = {0};
	
	char str_SetTempOffsetDisp1[10] = {0};
	char str_SetTempOffsetDisp2[10] = {0};
	char str_TempOffset1[10] = {0};
	char str_TempOffset2[10] = {0};
	
	char str_SystemWarningDisp[30];
	
	char str_AmbientTempDisp1[10] = {0};
	char str_AmbientTempDisp2[10] = {0};
	
	char str_DeviceTypeDisp[30];
	
	time_t T = time(NULL);
	struct  tm tm = *localtime(&T);
	sprintf(display_time,"%02d:%02d", tm.tm_hour, tm.tm_min);
	sprintf(display_date,"%02d.%02d.%d", tm.tm_mday, tm.tm_mon + 1, tm.tm_year - 100);
	
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->TimeLabel1), display_time);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->DateLabel1), display_date);
	
	sprintf(str_Current1, "%d", Current_Up_Temperature);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->RollTemp1), str_Current1);
	
	sprintf(str_Current2, "%d", Current_Down_Temperature);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->RollTemp2), str_Current2);
	
	sprintf(str_Speed, "%d", Speed);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SpeedEntry), str_Speed);
	
	sprintf(str_SetTempDisp1, "%d", Set_Temp_Value1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetTempDisp1), str_SetTempDisp1); 
	
	sprintf(str_SetTempDisp2, "%d", Set_Temp_Value2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetTempDisp2), str_SetTempDisp2);
	
	sprintf(str_Temp1, "%d", Temperature1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->TempEntry1), str_Temp1);
	
	sprintf(str_Temp2, "%d", Temperature2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->TempEntry2), str_Temp2);
	
	//---CtrlOffset---//
	sprintf(str_SetCtrlOffsetDisp1, "%d", Set_CtrlOffset_Value1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetCtrlOffsetDisp1), str_SetCtrlOffsetDisp1); 
	
	sprintf(str_SetCtrlOffsetDisp2, "%d", Set_CtrlOffset_Value2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetCtrlOffsetDisp2), str_SetCtrlOffsetDisp2);
	
	sprintf(str_CtrlOffset1, "%d", ControlOffset1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->CtrlOffsetEntry1), str_CtrlOffset1);
	
	sprintf(str_CtrlOffset2, "%d", ControlOffset2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->CtrlOffsetEntry2), str_CtrlOffset2);
	
	//---CtrlOutput---//
	sprintf(str_SetCtrlOutputDisp1, "%d", Set_CtrlOutput_Value1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetCtrlOutputDisp1), str_SetCtrlOutputDisp1); 
	
	sprintf(str_SetCtrlOutputDisp2, "%d", Set_CtrlOutput_Value2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetCtrlOutputDisp2), str_SetCtrlOutputDisp2);
	
	sprintf(str_CtrlOutput1, "%d", ControlOutput1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->CtrlOutputEntry1), str_CtrlOutput1);
	
	sprintf(str_CtrlOutput2, "%d", ControlOutput2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->CtrlOutputEntry2), str_CtrlOutput2);
	
	//---TempOffset---//
	sprintf(str_SetTempOffsetDisp1, "%d", Set_TempOffset_Value1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetTempOffsetDisp1), str_SetTempOffsetDisp1); 
	
	sprintf(str_SetTempOffsetDisp2, "%d", Set_TempOffset_Value2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SetTempOffsetDisp2), str_SetTempOffsetDisp2);
	
	sprintf(str_TempOffset1, "%d", TemperatureOffset1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->TempOffsetEntry1), str_TempOffset1);
	
	sprintf(str_TempOffset2, "%d", TemperatureOffset2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->TempOffsetEntry2), str_TempOffset2);
	
	if((LaserStatus) && (Laser))
	{
		gtk_widget_show(ThreadVal->Window->HandSafetyOFF);
	}
	else
	{
		gtk_widget_hide(ThreadVal->Window->HandSafetyOFF);
	}
	
	// Speed 
	if(PlusState)
	{
		gtk_widget_show(ThreadVal->Window->SpeedPlusDown);
		gtk_widget_hide(ThreadVal->Window->SpeedPlusUp);
		PlusState=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->SpeedPlusUp);
		gtk_widget_hide(ThreadVal->Window->SpeedPlusDown);
	}
	
	if(MinusState)
	{
		gtk_widget_show(ThreadVal->Window->SpeedMinusDown);
		gtk_widget_hide(ThreadVal->Window->SpeedMinusUp);
		MinusState=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->SpeedMinusUp);
		gtk_widget_hide(ThreadVal->Window->SpeedMinusDown);
	}
	
	// Temp1
	if(TempPlusState1)
	{
		gtk_widget_show(ThreadVal->Window->TempPlusDown1);
		gtk_widget_hide(ThreadVal->Window->TempPlusUp1);
		TempPlusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempPlusUp1);
		gtk_widget_hide(ThreadVal->Window->TempPlusDown1);
	}
	
	if(TempMinusState1)
	{
		gtk_widget_show(ThreadVal->Window->TempMinusDown1);
		gtk_widget_hide(ThreadVal->Window->TempMinusUp1);
		TempMinusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempMinusUp1);
		gtk_widget_hide(ThreadVal->Window->TempMinusDown1);
	}
	
	// Temp2
	if(TempPlusState2)
	{
		gtk_widget_show(ThreadVal->Window->TempPlusDown2);
		gtk_widget_hide(ThreadVal->Window->TempPlusUp2);
		TempPlusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempPlusUp2);
		gtk_widget_hide(ThreadVal->Window->TempPlusDown2);
	}
	
	if(TempMinusState2)
	{
		gtk_widget_show(ThreadVal->Window->TempMinusDown2);
		gtk_widget_hide(ThreadVal->Window->TempMinusUp2);
		TempMinusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempMinusUp2);
		gtk_widget_hide(ThreadVal->Window->TempMinusDown2);
	}
	
	if(SettingState)
	{
		gtk_widget_show(ThreadVal->Window->SettingButtonDown);
		gtk_widget_hide(ThreadVal->Window->SettingButtonUp);
		SettingState=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->SettingButtonUp);
		gtk_widget_hide(ThreadVal->Window->SettingButtonDown);
	}
	
	// CtrlOffset1
	if(CtrlOffsetPlusState1)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetPlusDown1);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetPlusUp1);
		CtrlOffsetPlusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetPlusUp1);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetPlusDown1);
	}
	
	if(CtrlOffsetMinusState1)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetMinusDown1);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetMinusUp1);
		CtrlOffsetMinusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetMinusUp1);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetMinusDown1);
	}
	
	// CtrlOffset2
	if(CtrlOffsetPlusState2)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetPlusDown2);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetPlusUp2);
		CtrlOffsetPlusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetPlusUp2);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetPlusDown2);
	}
	
	if(CtrlOffsetMinusState2)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetMinusDown2);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetMinusUp2);
		CtrlOffsetMinusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOffsetMinusUp2);
		gtk_widget_hide(ThreadVal->Window->CtrlOffsetMinusDown2);
	}
	
	// CtrlOutput1
	if(CtrlOutputPlusState1)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputPlusDown1);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputPlusUp1);
		CtrlOutputPlusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputPlusUp1);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputPlusDown1);
	}
	
	if(CtrlOutputMinusState1)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputMinusDown1);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputMinusUp1);
		CtrlOutputMinusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputMinusUp1);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputMinusDown1);
	}
	
	// CtrlOutput2
	if(CtrlOutputPlusState2)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputPlusDown2);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputPlusUp2);
		CtrlOutputPlusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputPlusUp2);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputPlusDown2);
	}
	
	if(CtrlOutputMinusState2)
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputMinusDown2);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputMinusUp2);
		CtrlOutputMinusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->CtrlOutputMinusUp2);
		gtk_widget_hide(ThreadVal->Window->CtrlOutputMinusDown2);
	}
	
	// TempOffset1
	if(TempOffsetPlusState1)
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetPlusDown1);
		gtk_widget_hide(ThreadVal->Window->TempOffsetPlusUp1);
		TempOffsetPlusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetPlusUp1);
		gtk_widget_hide(ThreadVal->Window->TempOffsetPlusDown1);
	}
	
	if(TempOffsetMinusState1)
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetMinusDown1);
		gtk_widget_hide(ThreadVal->Window->TempOffsetMinusUp1);
		TempOffsetMinusState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetMinusUp1);
		gtk_widget_hide(ThreadVal->Window->TempOffsetMinusDown1);
	}
	
	// TempOffset2
	if(TempOffsetPlusState2)
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetPlusDown2);
		gtk_widget_hide(ThreadVal->Window->TempOffsetPlusUp2);
		TempOffsetPlusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetPlusUp2);
		gtk_widget_hide(ThreadVal->Window->TempOffsetPlusDown2);
	}
	
	if(TempOffsetMinusState2)
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetMinusDown2);
		gtk_widget_hide(ThreadVal->Window->TempOffsetMinusUp2);
		TempOffsetMinusState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->TempOffsetMinusUp2);
		gtk_widget_hide(ThreadVal->Window->TempOffsetMinusDown2);
	}
	
	//SystemWarning
	//memset(&str_SystemWarningDisp[0],0,sizeof(str_SystemWarningDisp));
	sprintf(str_SystemWarningDisp, "%s", &u8AlarmStr[SystemWarning][0]);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->SystemWarningDisp), str_SystemWarningDisp); 
	
	//AmbientTemperature	
	//AmbientTemperature1
	sprintf(str_AmbientTempDisp1, "%d", AmbientTemperature1);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->AmbientTempDisp1), str_AmbientTempDisp1); 
	
	//AmbientTemperature2
	sprintf(str_AmbientTempDisp2, "%d", AmbientTemperature2);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->AmbientTempDisp2), str_AmbientTempDisp2);
	
	//DeviceType
	//memset(&str_DeviceTypeDisp[0],0,sizeof(str_DeviceTypeDisp));
	sprintf(str_DeviceTypeDisp, "%s", &Device[DeviceType][0]);
	gtk_label_set_text(GTK_LABEL(ThreadVal->Window->DeviceTypeDisp), str_DeviceTypeDisp);
	
	// Num1
	if(NumState1)
	{
		gtk_widget_show(ThreadVal->Window->Dwn1);
		gtk_widget_hide(ThreadVal->Window->Up1);
		NumState1=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up1);
		gtk_widget_hide(ThreadVal->Window->Dwn1);
	}
	
	// Num2
	if(NumState2)
	{
		gtk_widget_show(ThreadVal->Window->Dwn2);
		gtk_widget_hide(ThreadVal->Window->Up2);
		NumState2=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up2);
		gtk_widget_hide(ThreadVal->Window->Dwn2);
	}
	
	// Num3
	if(NumState3)
	{
		gtk_widget_show(ThreadVal->Window->Dwn3);
		gtk_widget_hide(ThreadVal->Window->Up3);
		NumState3=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up3);
		gtk_widget_hide(ThreadVal->Window->Dwn3);
	}
	
	// Num4
	if(NumState4)
	{
		gtk_widget_show(ThreadVal->Window->Dwn4);
		gtk_widget_hide(ThreadVal->Window->Up4);
		NumState4=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up4);
		gtk_widget_hide(ThreadVal->Window->Dwn4);
	}
	
	// Num5
	if(NumState5)
	{
		gtk_widget_show(ThreadVal->Window->Dwn5);
		gtk_widget_hide(ThreadVal->Window->Up5);
		NumState5=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up5);
		gtk_widget_hide(ThreadVal->Window->Dwn5);
	}
	
	// Num6
	if(NumState6)
	{
		gtk_widget_show(ThreadVal->Window->Dwn6);
		gtk_widget_hide(ThreadVal->Window->Up6);
		NumState6=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up6);
		gtk_widget_hide(ThreadVal->Window->Dwn6);
	}
	
	// Num7
	if(NumState7)
	{
		gtk_widget_show(ThreadVal->Window->Dwn7);
		gtk_widget_hide(ThreadVal->Window->Up7);
		NumState7=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up7);
		gtk_widget_hide(ThreadVal->Window->Dwn7);
	}
	
	// Num8
	if(NumState8)
	{
		gtk_widget_show(ThreadVal->Window->Dwn8);
		gtk_widget_hide(ThreadVal->Window->Up8);
		NumState8=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up8);
		gtk_widget_hide(ThreadVal->Window->Dwn8);
	}
	
	// Num9
	if(NumState9)
	{
		gtk_widget_show(ThreadVal->Window->Dwn9);
		gtk_widget_hide(ThreadVal->Window->Up9);
		NumState9=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up9);
		gtk_widget_hide(ThreadVal->Window->Dwn9);
	}
	
	// Num0
	if(NumState0)
	{
		gtk_widget_show(ThreadVal->Window->Dwn0);
		gtk_widget_hide(ThreadVal->Window->Up0);
		NumState0=0;
	}
	else
	{
		gtk_widget_show(ThreadVal->Window->Up0);
		gtk_widget_hide(ThreadVal->Window->Dwn0);
	}
	
	switch(passwordlen)
	{
		case 0:
			gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry1), " ");
			gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry2), " ");
			gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry3), " ");
			gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry4), " ");
			break;
		case 1: gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry1), "*"); 
			break;
		case 2: gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry2), "*"); 
			break;
		case 3: gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry3), "*"); 
			break;
		case 4: gtk_label_set_text(GTK_LABEL(ThreadVal->Window->PinEntry4), "*"); 
			break;
	} 
	
	return G_SOURCE_CONTINUE;
}

int main(int argc, char *argv[])
{
	Thread_Arguments  ThreadVal;
	pthread_t mSock_tid;
	
	// instantiate structure, allocating memory for it
	app_widgets        *widgets = g_slice_new(app_widgets);
	
    gtk_init(&argc, &argv);

    builder = gtk_builder_new();
    gtk_builder_add_from_file (builder, "LaminationMachine.glade", NULL);
	
	HomeWindow = GTK_WIDGET(gtk_builder_get_object(builder, "HomeWindow"));
	HomeWlayout = GTK_WIDGET(gtk_builder_get_object(builder, "HomeWlayout"));
	Background1 = GTK_WIDGET(gtk_builder_get_object(builder, "Background1"));
	TimeIcon1 = GTK_WIDGET(gtk_builder_get_object(builder, "TimeIcon1"));
	XLJetHeader1 = GTK_WIDGET(gtk_builder_get_object(builder, "XLJetHeader1"));
	DateIcon1 = GTK_WIDGET(gtk_builder_get_object(builder, "DateIcon1"));
	widgets->TimeLabel1 = GTK_WIDGET(gtk_builder_get_object(builder, "TimeLabel1"));
	widgets->DateLabel1 = GTK_WIDGET(gtk_builder_get_object(builder, "DateLabel1"));
	widgets->RollTemp1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollTemp1"));
	widgets->RollTemp2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollTemp2"));
	widgets->SetTempDisp1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempDisp1"));
	widgets->SetTempDisp2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempDisp2"));
	HeaterToggle1 = GTK_WIDGET(gtk_builder_get_object(builder, "HeaterToggle1"));
	HeaterToggle2 = GTK_WIDGET(gtk_builder_get_object(builder, "HeaterToggle2"));
	widgets->SpeedEntry = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedEntry"));
	MinusButton = GTK_WIDGET(gtk_builder_get_object(builder, "MinusButton"));
	PlusButton = GTK_WIDGET(gtk_builder_get_object(builder, "PlusButton"));
	SettingsButton = GTK_WIDGET(gtk_builder_get_object(builder, "SettingsButton"));
	
	HandSafetyToggle = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "HandSafetyToggle"));
	MotorRButton = GTK_WIDGET(gtk_builder_get_object(builder, "MotorRButton"));
	MotorFButton = GTK_WIDGET(gtk_builder_get_object(builder, "MotorFButton"));
	StartButton = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "StartButton"));
	
	RollerDownButton = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "RollerDownButton"));
	RollerUpButton = GTK_TOGGLE_BUTTON(gtk_builder_get_object(builder, "RollerUpButton"));

	RollerFrame1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerFrame1"));
	RollerFrame2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerFrame2"));
	RollerText1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerText1"));
	RollerText2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerText2"));
	DegreeBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "DegreeBox1"));
	DegreeBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "DegreeBox2"));
	SetBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetBox1"));
	SetBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetBox2"));
	RollerHeaterON1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerHeaterON1"));
	RollerHeaterOFF1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerHeaterOFF1"));
	RollerHeaterON2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerHeaterON2"));
	RollerHeaterOFF2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerHeaterOFF2"));
	SpeedText = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedText"));
	widgets->SpeedMinusDown = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedMinusDown"));
	widgets->SpeedMinusUp = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedMinusUp"));
	SpeedDataBox = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedDataBox"));
	widgets->SpeedPlusDown = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedPlusDown"));
	widgets->SpeedPlusUp = GTK_WIDGET(gtk_builder_get_object(builder, "SpeedPlusUp"));
	HandSafetyON = GTK_WIDGET(gtk_builder_get_object(builder, "HandSafetyON"));
	widgets->HandSafetyOFF = GTK_WIDGET(gtk_builder_get_object(builder, "HandSafetyOFF"));
	HandSafetyDisable = GTK_WIDGET(gtk_builder_get_object(builder, "HandSafetyDisable"));
	RollerUpON = GTK_WIDGET(gtk_builder_get_object(builder, "RollerUpON"));
	RollerUpOFF = GTK_WIDGET(gtk_builder_get_object(builder, "RollerUpOFF"));
	widgets->SettingButtonUp = GTK_WIDGET(gtk_builder_get_object(builder, "SettingButtonUp"));
	widgets->SettingButtonDown = GTK_WIDGET(gtk_builder_get_object(builder, "SettingButtonDown"));
	ReverseON = GTK_WIDGET(gtk_builder_get_object(builder, "ReverseON"));
	ReverseOFF = GTK_WIDGET(gtk_builder_get_object(builder, "ReverseOFF"));
	RollerDownON = GTK_WIDGET(gtk_builder_get_object(builder, "RollerDownON"));
	RollerDownOFF = GTK_WIDGET(gtk_builder_get_object(builder, "RollerDownOFF"));
	ForwardON = GTK_WIDGET(gtk_builder_get_object(builder, "ForwardON"));
	ForwardOFF = GTK_WIDGET(gtk_builder_get_object(builder, "ForwardOFF"));
	StartImage = GTK_WIDGET(gtk_builder_get_object(builder, "StartImage"));
	StopImage = GTK_WIDGET(gtk_builder_get_object(builder, "StopImage"));
	
	Temp1 = GTK_WIDGET(gtk_builder_get_object(builder, "Temp1"));
	TempLayout1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempLayout1"));
	TempPlusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusButton1"));
	TempMinusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusButton1"));
	TempOKButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOKButton1"));
	TempCancelButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempCancelButton1"));
	PopUpBG1 = GTK_WIDGET(gtk_builder_get_object(builder, "PopUpBG1"));
	RollerUpPopup1 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerUpPopup1"));
	SetTempPopup1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempPopup1"));
	widgets->TempMinusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusDown1"));
	widgets->TempMinusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusUp1"));
	DataBoxPopup1 = GTK_WIDGET(gtk_builder_get_object(builder, "DataBoxPopup1"));
	widgets->TempPlusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusDown1"));
	widgets->TempPlusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusUp1"));
	OKPopup1 = GTK_WIDGET(gtk_builder_get_object(builder, "OKPopup1"));
	CancelPopup1 = GTK_WIDGET(gtk_builder_get_object(builder, "CancelPopup1"));
	widgets->TempEntry1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempEntry1"));
	
	Temp2 = GTK_WIDGET(gtk_builder_get_object(builder, "Temp2"));
	TempLayout2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempLayout2"));
	TempPlusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusButton2"));
	TempMinusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusButton2"));
	TempOKButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOKButton2"));
	TempCancelButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempCancelButton2"));
	PopUpBG2 = GTK_WIDGET(gtk_builder_get_object(builder, "PopUpBG2"));
	RollerUpPopup2 = GTK_WIDGET(gtk_builder_get_object(builder, "RollerUpPopup2"));
	SetTempPopup2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempPopup2"));
	widgets->TempMinusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusDown2"));
	widgets->TempMinusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempMinusUp2"));
	DataBoxPopup2 = GTK_WIDGET(gtk_builder_get_object(builder, "DataBoxPopup2"));
	widgets->TempPlusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusDown2"));
	widgets->TempPlusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempPlusUp2"));
	OKPopup2 = GTK_WIDGET(gtk_builder_get_object(builder, "OKPopup2"));
	CancelPopup2 = GTK_WIDGET(gtk_builder_get_object(builder, "CancelPopup2"));
	widgets->TempEntry2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempEntry2"));
	
	MenuWindow = GTK_WIDGET(gtk_builder_get_object(builder, "MenuWindow"));

	Background2 = GTK_WIDGET(gtk_builder_get_object(builder, "Background2"));
	XLJetHeader2 = GTK_WIDGET(gtk_builder_get_object(builder, "XLJetHeader2"));
	
	CalibrationBImage = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationBImage"));
	DeviceStatusBImage = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceStatusBImage"));
	MenuExitBImage = GTK_WIDGET(gtk_builder_get_object(builder, "MenuExitBImage"));
	CalibrationSButton = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationSButton"));
	DeviceSButton = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceSButton"));
	MenuExitButton = GTK_WIDGET(gtk_builder_get_object(builder, "MenuExitButton"));
	
	CalibrationSWindow = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationSWindow"));
	
	SetCtrlOffsetUp = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOffsetUp"));
	SetCtrlOffsetDwn = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOffsetDwn"));
	SetCtrlOutputUp = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOutputUp"));
	SetCtrlOutputDwn = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOutputDwn"));
	SetTempOffsetUp = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempOffsetUp"));
	SetTempOffsetDwn = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempOffsetDwn"));
	CalibrationExitButton = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationExitButton"));
	Background3 = GTK_WIDGET(gtk_builder_get_object(builder, "Background3"));
	XLJetHeader3 = GTK_WIDGET(gtk_builder_get_object(builder, "XLJetHeader3"));
	ControlOffsetBox = GTK_WIDGET(gtk_builder_get_object(builder, "ControlOffsetBox"));
	CtrlOffsetUpLabel = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetUpLabel"));
	CtrlOffsetDwnLabel = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetDwnLabel"));
	CtrlOffsetUpImg = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetUpImg"));
	CtrlOffsetDwnImg = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetDwnImg"));
	ControlOutputBox = GTK_WIDGET(gtk_builder_get_object(builder, "ControlOutputBox"));
	CtrlOutputUpLabel = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputUpLabel"));
	CtrlOutputDwnLabel = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputDwnLabel"));
	CtrlOutputUpImg = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputUpImg"));
	CtrlOutputDwnImg = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputDwnImg"));
	TempOffsetBox = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetBox"));
	TempOffsetUpLabel = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetUpLabel"));
	TempOffsetDwnLabel = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetDwnLabel"));
	TempOffsetUpImg = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetUpImg"));
	TmpOffsetDwnEntry = GTK_WIDGET(gtk_builder_get_object(builder, "TmpOffsetDwnEntry"));
	CalibrationExitImage = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationExitImage"));
	widgets->SetCtrlOffsetDisp1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOffsetDisp1"));
	widgets->SetCtrlOffsetDisp2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOffsetDisp2"));
	widgets->SetCtrlOutputDisp1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOutputDisp1"));
	widgets->SetCtrlOutputDisp2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetCtrlOutputDisp2"));
	widgets->SetTempOffsetDisp1 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempOffsetDisp1"));
	widgets->SetTempOffsetDisp2 = GTK_WIDGET(gtk_builder_get_object(builder, "SetTempOffsetDisp2"));
	
	CtrlOffset1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffset1"));
	
	CtrlOffsetMinusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusButton1"));
	CtrlOffsetCancelButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetCancelButton1"));
	CtrlOffsetOKButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetOKButton1"));
	CtrlOffsetPlusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusButton1"));
	CtrlOffsetPopUpBG1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPopUpBG1"));
	CtrlOffsetUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetUp1"));
	CtrlOffsetPop1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPop1"));
	widgets->CtrlOffsetMinusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusDown1"));
	widgets->CtrlOffsetMinusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusUp1"));
	CtrlOffsetBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetBox1"));
	widgets->CtrlOffsetPlusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusDown1"));
	widgets->CtrlOffsetPlusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusUp1"));
	CtrlOffsetOK1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetOK1"));
	CtrlOffsetCancel1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetCancel1"));
	widgets->CtrlOffsetEntry1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetEntry1"));
	
	CtrlOffset2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffset2"));
	
	CtrlOffsetMinusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusButton2"));
	CtrlOffsetCancelButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetCancelButton2"));
	CtrlOffsetOKButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetOKButton2"));
	CtrlOffsetPlusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusButton2"));
	CtrlOffsetPopUpBG2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPopUpBG2"));
	CtrlOffsetUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetUp2"));
	CtrlOffsetPop2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPop2"));
	widgets->CtrlOffsetMinusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusDown2"));
	widgets->CtrlOffsetMinusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetMinusUp2"));
	CtrlOffsetBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetBox2"));
	widgets->CtrlOffsetPlusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusDown2"));
	widgets->CtrlOffsetPlusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetPlusUp2"));
	CtrlOffsetOK2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetOK2"));
	CtrlOffsetCancel2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetCancel2"));
	widgets->CtrlOffsetEntry2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOffsetEntry2"));
	
	CtrlOutput1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutput1"));
	
	CtrlOutputMinusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusButton1"));
	CtrlOutputCancelButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputCancelButton1"));
	CtrlOutputOKButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputOKButton1"));
	CtrlOutputPlusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusButton1"));
	CtrlOutputPopUpBG1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPopUpBG1"));
	CtrlOutputUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputUp1"));
	CtrlOutputPop1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPop1"));
	widgets->CtrlOutputMinusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusDown1"));
	widgets->CtrlOutputMinusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusUp1"));
	CtrlOutputBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputBox1"));
	widgets->CtrlOutputPlusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusDown1"));
	widgets->CtrlOutputPlusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusUp1"));
	CtrlOutputOK1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputOK1"));
	CtrlOutputCancel1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputCancel1"));
	widgets->CtrlOutputEntry1 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputEntry1"));
	
	CtrlOutput2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutput2"));
	
	CtrlOutputMinusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusButton2"));
	CtrlOutputCancelButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputCancelButton2"));
	CtrlOutputOKButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputOKButton2"));
	CtrlOutputPlusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusButton2"));
	CtrlOutputPopUpBG2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPopUpBG2"));
	CtrlOutputUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputUp2"));
	CtrlOutputPop2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPop2"));
	widgets->CtrlOutputMinusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusDown2"));
	widgets->CtrlOutputMinusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputMinusUp2"));
	CtrlOutputBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputBox2"));
	widgets->CtrlOutputPlusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusDown2"));
	widgets->CtrlOutputPlusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputPlusUp2"));
	CtrlOutputOK2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputOK2"));
	CtrlOutputCancel2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputCancel2"));
	widgets->CtrlOutputEntry2 = GTK_WIDGET(gtk_builder_get_object(builder, "CtrlOutputEntry2"));
	
	TempOffset1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffset1"));
	
	TempOffsetMinusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusButton1"));
	TempOffsetCancelButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetCancelButton1"));
	TempOffsetOKButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetOKButton1"));
	TempOffsetPlusButton1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusButton1"));
	TempOffsetPopUpBG1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPopUpBG1"));
	TempOffsetUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetUp1"));
	TempOffsetPop1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPop1"));
	widgets->TempOffsetMinusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusDown1"));
	widgets->TempOffsetMinusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusUp1"));
	TempOffsetBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetBox1"));
	widgets->TempOffsetPlusDown1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusDown1"));
	widgets->TempOffsetPlusUp1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusUp1"));
	TempOffsetOK1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetOK1"));
	TempOffsetCancel1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetCancel1"));
	widgets->TempOffsetEntry1 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetEntry1"));
	
	TempOffset2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffset2"));
	
	TempOffsetMinusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusButton2"));
	TempOffsetCancelButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetCancelButton2"));
	TempOffsetOKButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetOKButton2"));
	TempOffsetPlusButton2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusButton2"));
	TempOffsetPopUpBG2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPopUpBG2"));
	TempOffsetUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetUp2"));
	TempOffsetPop2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPop2"));
	widgets->TempOffsetMinusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusDown2"));
	widgets->TempOffsetMinusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetMinusUp2"));
	TempOffsetBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetBox2"));
	widgets->TempOffsetPlusDown2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusDown2"));
	widgets->TempOffsetPlusUp2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetPlusUp2"));
	TempOffsetOK2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetOK2"));
	TempOffsetCancel2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetCancel2"));
	widgets->TempOffsetEntry2 = GTK_WIDGET(gtk_builder_get_object(builder, "TempOffsetEntry2"));
	
	DeviceStatusWindow = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceStatusWindow"));
	
	DeviceStatusExitButton = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceStatusExitButton"));
	Background4 = GTK_WIDGET(gtk_builder_get_object(builder, "Background4"));
	XLJetHeader4 = GTK_WIDGET(gtk_builder_get_object(builder, "XLJetHeader4"));
	SystemWarningBox = GTK_WIDGET(gtk_builder_get_object(builder, "SystemWarningBox"));
	AmbientTempBox = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTempBox"));
	AmbientTempUpLabel = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTempUpLabel"));
	AmbientTempDownLabel = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTempDownLabel"));
	AmbientTmpUpImg = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTmpUpImg"));
	AmbientTmpDwnImg = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTmpDwnImg"));
	DeviceTypeBox = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceTypeBox"));
	widgets->SystemWarningDisp = GTK_WIDGET(gtk_builder_get_object(builder, "SystemWarningDisp"));
	widgets->AmbientTempDisp1 = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTempDisp1"));
	widgets->AmbientTempDisp2 = GTK_WIDGET(gtk_builder_get_object(builder, "AmbientTempDisp2"));
	widgets->DeviceTypeDisp = GTK_WIDGET(gtk_builder_get_object(builder, "DeviceTypeDisp"));
	
	PasswordWindow = GTK_WIDGET(gtk_builder_get_object(builder, "PasswordWindow"));
	
	Num1 = GTK_WIDGET(gtk_builder_get_object(builder, "Num1"));
	Num2 = GTK_WIDGET(gtk_builder_get_object(builder, "Num2"));
	Num3 = GTK_WIDGET(gtk_builder_get_object(builder, "Num3"));
	Num4 = GTK_WIDGET(gtk_builder_get_object(builder, "Num4"));
	Num5 = GTK_WIDGET(gtk_builder_get_object(builder, "Num5"));
	Num6 = GTK_WIDGET(gtk_builder_get_object(builder, "Num6"));
	Num7 = GTK_WIDGET(gtk_builder_get_object(builder, "Num7"));
	Num8 = GTK_WIDGET(gtk_builder_get_object(builder, "Num8"));
	Num9 = GTK_WIDGET(gtk_builder_get_object(builder, "Num9"));
	Num0 = GTK_WIDGET(gtk_builder_get_object(builder, "Num0"));
	DeleteButton = GTK_WIDGET(gtk_builder_get_object(builder, "DeleteButton"));
	BackButton = GTK_WIDGET(gtk_builder_get_object(builder, "BackButton"));
	EnterButton = GTK_WIDGET(gtk_builder_get_object(builder, "EnterButton"));
	Background5 = GTK_WIDGET(gtk_builder_get_object(builder, "Background5"));
	XLJetHeader5 = GTK_WIDGET(gtk_builder_get_object(builder, "XLJetHeader5"));
	EnterPin = GTK_WIDGET(gtk_builder_get_object(builder, "EnterPin"));
	PinEntryBox1 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntryBox1"));
	PinEntryBox2 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntryBox2"));
	PinEntryBox3 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntryBox3"));
	PinEntryBox4 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntryBox4"));
	widgets->Dwn1 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn1"));
	widgets->Dwn2 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn2"));
	widgets->Dwn3 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn3"));
	widgets->Dwn4 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn4"));
	widgets->Dwn5 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn5"));
	widgets->Dwn6 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn6"));
	widgets->Dwn7 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn7"));
	widgets->Dwn8 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn8"));
	widgets->Dwn9 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn9"));
	widgets->Dwn0 = GTK_WIDGET(gtk_builder_get_object(builder, "Dwn0"));
	widgets->Up1 = GTK_WIDGET(gtk_builder_get_object(builder, "Up1"));
	widgets->Up2 = GTK_WIDGET(gtk_builder_get_object(builder, "Up2"));
	widgets->Up3 = GTK_WIDGET(gtk_builder_get_object(builder, "Up3"));
	widgets->Up4 = GTK_WIDGET(gtk_builder_get_object(builder, "Up4"));
	widgets->Up5 = GTK_WIDGET(gtk_builder_get_object(builder, "Up5"));
	widgets->Up6 = GTK_WIDGET(gtk_builder_get_object(builder, "Up6"));
	widgets->Up7 = GTK_WIDGET(gtk_builder_get_object(builder, "Up7"));
	widgets->Up8 = GTK_WIDGET(gtk_builder_get_object(builder, "Up8"));
	widgets->Up9 = GTK_WIDGET(gtk_builder_get_object(builder, "Up9"));
	widgets->Up0 = GTK_WIDGET(gtk_builder_get_object(builder, "Up0"));
	widgets->PinEntry1 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntry1"));
	widgets->PinEntry2 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntry2"));
	widgets->PinEntry3 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntry3"));
	widgets->PinEntry4 = GTK_WIDGET(gtk_builder_get_object(builder, "PinEntry4"));
	BackImg = GTK_WIDGET(gtk_builder_get_object(builder, "BackImg"));
	EnterImg = GTK_WIDGET(gtk_builder_get_object(builder, "EnterImg"));
	
	PwdCheckWindow = GTK_WIDGET(gtk_builder_get_object(builder, "PwdCheckWindow"));
	PwdOKButton = GTK_WIDGET(gtk_builder_get_object(builder, "PwdOKButton"));
	WrongPin_BG = GTK_WIDGET(gtk_builder_get_object(builder, "WrongPin_BG"));
	OkWrongBG = GTK_WIDGET(gtk_builder_get_object(builder, "OkWrongBG"));
	
	HomeBlurWindow = GTK_WIDGET(gtk_builder_get_object(builder, "HomeBlurWindow"));
	HomeBlur = GTK_WIDGET(gtk_builder_get_object(builder, "HomeBlur"));
	
	CalibrationBlurWindow = GTK_WIDGET(gtk_builder_get_object(builder, "CalibrationBlurWindow"));
	CalibrationBlur = GTK_WIDGET(gtk_builder_get_object(builder, "HomeBlur"));
	
	PasswordBlurWindow = GTK_WIDGET(gtk_builder_get_object(builder, "PasswordBlurWindow"));
	PasswordBlur = GTK_WIDGET(gtk_builder_get_object(builder, "PasswordBlur"));
	
	ThreadVal.Window = widgets;
	ThreadVal.Status = 1;
	
	pthread_create (&mSock_tid, NULL, Data_Background_thread, NULL);
	
	g_timeout_add (100 /* milliseconds */, on_timeout, &ThreadVal);
	
	gtk_builder_connect_signals(builder, widgets);    // note: second parameter is not NULL.
	
    g_object_unref(builder);
	
    gtk_widget_show(HomeWindow);
	
    gtk_main();
	
    return 0;
}

void on_SetTempButton1_clicked()
{
	gtk_widget_show(HomeBlurWindow);
	gtk_widget_show(Temp1);
}

void on_TempPlusButton1_clicked()
{
	TempPlusState1=1;
	if (Temperature1 < 150)
	{
		Temperature1++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempMinusButton1_clicked()
{
	TempMinusState1=1;
	if (Temperature1 > 30)
	{
		Temperature1--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOKButton1_clicked()
{
	Set_Temp_Value1 = Temperature1;
	gtk_widget_hide(HomeBlurWindow);
	gtk_widget_hide(Temp1);
}

void on_TempCancelButton1_clicked()
{
	gtk_widget_hide(HomeBlurWindow);
	gtk_widget_hide(Temp1);
}

void on_SetTempButton2_clicked()
{
	gtk_widget_show(HomeBlurWindow);
	gtk_widget_show(Temp2);
}

void on_TempPlusButton2_clicked()
{
	TempPlusState2=1;
	if (Temperature2 < 150)
	{
		Temperature2++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempMinusButton2_clicked()
{
	TempMinusState2=1;
	if (Temperature2 > 30)
	{
		Temperature2--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOKButton2_clicked()
{
	Set_Temp_Value2 = Temperature2;
	gtk_widget_hide(HomeBlurWindow);
	gtk_widget_hide(Temp2);
}

void on_TempCancelButton2_clicked()
{
	gtk_widget_hide(HomeBlurWindow);
	gtk_widget_hide(Temp2);
}

void	on_HeaterToggle1_toggled(GtkToggleButton *b) {
	gboolean T = gtk_toggle_button_get_active(b);
	if (T)
	{
		gtk_widget_show(RollerHeaterON1);
		gtk_widget_hide(RollerHeaterOFF1);
		Heater1 = 1;
		STDregisters.Heater_ON_OFF_Up_Sensor = Heater1;
	}
		
	else 
	{
		gtk_widget_show(RollerHeaterOFF1);
		gtk_widget_hide(RollerHeaterON1);
		Heater1 = 0;
		STDregisters.Heater_ON_OFF_Up_Sensor = Heater1;
	}
}

void	on_HeaterToggle2_toggled(GtkToggleButton *b) {
	gboolean T = gtk_toggle_button_get_active(b);
	if (T)
	{
		gtk_widget_show(RollerHeaterON2);
		gtk_widget_hide(RollerHeaterOFF2);
		Heater2 = 1;
		STDregisters.Heater_ON_OFF_Down_Sensor = Heater2;
	}
		
	else 
	{
		gtk_widget_show(RollerHeaterOFF2);
		gtk_widget_hide(RollerHeaterON2);
		Heater2 = 0;
		STDregisters.Heater_ON_OFF_Down_Sensor = Heater2;
	}
}

void on_PlusButton_clicked()
{
	PlusState=1;
	if (Speed < 100)
	{
		Speed = Speed + 1;
		STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_MinusButton_clicked()
{
	MinusState=1;
	if (Speed > 10)
	{
		Speed = Speed - 1;
		STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void	on_HandSafetyToggle_toggled(GtkToggleButton *b) {
	gboolean T = gtk_toggle_button_get_active(b);
	if (T)
	{
		gtk_widget_show(HandSafetyON);
		gtk_widget_hide(HandSafetyDisable);
		Laser = 1;
		STDregisters.Laser_Control_Reg = Laser;
	}
	else
	{
		gtk_widget_show(HandSafetyDisable);
		gtk_widget_hide(HandSafetyON);
		Laser = 0;
		STDregisters.Laser_Control_Reg = Laser;
	}
}

void	on_RollerUpButton_toggled(GtkToggleButton *b) 
{
	RollerUpState = gtk_toggle_button_get_active(b);                                                                  
	if (RollerUpState)
	{
		gtk_widget_show(RollerUpON);
		gtk_widget_hide(RollerUpOFF);
		gtk_toggle_button_set_active (RollerDownButton,
                              FALSE);
		gtk_widget_show(RollerDownOFF);
		gtk_widget_hide(RollerDownON);
		RollerDirection = ROLLER_UP;
		STDregisters.Rod_Up_Down = RollerDirection;
	}
	else 
	{
		gtk_widget_show(RollerUpOFF);
		gtk_widget_hide(RollerUpON);
		RollerDirection = ROLLER_HALT;
		STDregisters.Rod_Up_Down = RollerDirection;
	}
}

void	on_RollerDownButton_toggled(GtkToggleButton *b) 
{
	RollerDownState = gtk_toggle_button_get_active(b);                                                                  
	if (RollerDownState)
	{
		gtk_widget_show(RollerDownON);
		gtk_widget_hide(RollerDownOFF);
		gtk_toggle_button_set_active (RollerUpButton,
                              FALSE);
		gtk_widget_show(RollerUpOFF);
		gtk_widget_hide(RollerUpON);
		RollerDirection = ROLLER_DWN;
		STDregisters.Rod_Up_Down = RollerDirection;
	}
	else
	{
		gtk_widget_show(RollerDownOFF);
		gtk_widget_hide(RollerDownON);
		RollerDirection = ROLLER_HALT;
		STDregisters.Rod_Up_Down = RollerDirection;
	}
}

void on_MotorRButton_clicked()
{
	MotorDirection=0;
	gtk_widget_hide(ReverseOFF);
	gtk_widget_show(ReverseON);
	gtk_widget_hide(ForwardON);
	gtk_widget_show(ForwardOFF);
	STDregisters.Rod_Up_Down=MotorDirection;
}

void on_MotorFButton_clicked()
{
	MotorDirection=1;
	gtk_widget_show(ForwardOFF);
	gtk_widget_show(ForwardON);
	gtk_widget_hide(ReverseON);
	gtk_widget_show(ReverseOFF);
	STDregisters.Rod_Up_Down=MotorDirection;
}

void	on_StartButton_toggled(GtkToggleButton *b) {
	gboolean T = gtk_toggle_button_get_active(b);
	if (T)
	{
		gtk_widget_show(StartImage);
		gtk_widget_hide(StopImage);
		MotorState=1;
		STDregisters.Motor_Start_Stop = MotorState;
		
	}
	else 
	{
		gtk_widget_show(StopImage);
		gtk_widget_hide(StartImage);
		MotorState=0;
		STDregisters.Motor_Start_Stop = MotorState;
	}
}

//---------------------------------------------------Settings Button & Page---------------------------------------------------//

void on_MenuExitButton_clicked()
{
	gtk_widget_show(HomeWindow);
	usleep(10000);
	gtk_widget_hide(MenuWindow);
	usleep(10000);
}
//-----------------------------------------------------Device Status Page------------------------------------------------------//

void on_DeviceSButton_clicked()
{
	gtk_widget_show(DeviceStatusWindow);
	usleep(10000);
	gtk_widget_hide(MenuWindow);
	usleep(10000);
}

void on_DeviceStatusExitButton_clicked()
{
	gtk_widget_show(HomeWindow);
	usleep(10000);
	gtk_widget_hide(DeviceStatusWindow);
	usleep(10000);
}

//-------------------------------------------------Calibration Button & Page--------------------------------------------------//

void on_CalibrationSButton_clicked()
{
	gtk_widget_show(CalibrationSWindow);
	usleep(10000);
	gtk_widget_hide(MenuWindow);
	usleep(10000);
}

void on_CalibrationExitButton_clicked()
{
	gtk_widget_show(HomeWindow);
	usleep(10000);
	gtk_widget_hide(CalibrationSWindow);
	usleep(10000);
}

//CtrlOffset
void on_SetCtrlOffsetUp_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(CtrlOffset1);
}

void on_SetCtrlOffsetDwn_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(CtrlOffset2);
}

//CtrlOutput
void on_SetCtrlOutputUp_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(CtrlOutput1);
}

void on_SetCtrlOutputDwn_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(CtrlOutput2);
}

//TempOffset
void on_SetTempOffsetUp_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(TempOffset1);
}

void on_SetTempOffsetDwn_clicked()
{
	gtk_widget_show(CalibrationBlurWindow);
	gtk_widget_show(TempOffset2);
}

//CtrlOffset
//CtrlOffsetUp
void on_CtrlOffsetMinusButton1_clicked()
{
	CtrlOffsetMinusState1=1;
	if (ControlOffset1 > 0)
	{
		ControlOffset1--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOffsetPlusButton1_clicked()
{
	CtrlOffsetPlusState1=1;
	if (ControlOffset1 < 20)
	{
		ControlOffset1++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOffsetOKButton1_clicked()
{
	Set_CtrlOffset_Value1 = ControlOffset1;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOffset1);
}

void on_CtrlOffsetCancelButton1_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOffset1);
}

//CtrlOffsetDwn
void on_CtrlOffsetMinusButton2_clicked()
{
	CtrlOffsetMinusState2=1;
	if (ControlOffset2 > 0)
	{
		ControlOffset2--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOffsetPlusButton2_clicked()
{
	CtrlOffsetPlusState2=1;
	if (ControlOffset2 < 20)
	{
		ControlOffset2++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOffsetOKButton2_clicked()
{
	Set_CtrlOffset_Value2 = ControlOffset2;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOffset2);
}

void on_CtrlOffsetCancelButton2_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOffset2);
}

//CtrlOutput
//CtrlOutputUp
void on_CtrlOutputMinusButton1_clicked()
{
	CtrlOutputMinusState1=1;
	if (ControlOutput1 > 10)
	{
		ControlOutput1--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOutputPlusButton1_clicked()
{
	CtrlOutputPlusState1=1;
	if (ControlOutput1 < 100)
	{
		ControlOutput1++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOutputOKButton1_clicked()
{
	Set_CtrlOutput_Value1 = ControlOutput1;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOutput1);
}

void on_CtrlOutputCancelButton1_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOutput1);
}

//CtrlOutputDwn
void on_CtrlOutputMinusButton2_clicked()
{
	CtrlOutputMinusState2=1;
	if (ControlOutput2 > 10)
	{
		ControlOutput2--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOutputPlusButton2_clicked()
{
	CtrlOutputPlusState2=1;
	if (ControlOutput2 < 100)
	{
		ControlOutput2++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_CtrlOutputOKButton2_clicked()
{
	Set_CtrlOutput_Value2 = ControlOutput2;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOutput2);
}

void on_CtrlOutputCancelButton2_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(CtrlOutput2);
}

//TempOffset
//TempOffsetUp
void on_TempOffsetMinusButton1_clicked()
{
	TempOffsetMinusState1=1;
	if (TemperatureOffset1 > -25)
	{
		TemperatureOffset1--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOffsetPlusButton1_clicked()
{
	TempOffsetPlusState1=1;
	if (TemperatureOffset1 < 25)
	{
		TemperatureOffset1++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOffsetOKButton1_clicked()
{
	Set_TempOffset_Value1 = TemperatureOffset1;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(TempOffset1);
}

void on_TempOffsetCancelButton1_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(TempOffset1);
}

//TempOffsetDwn
void on_TempOffsetMinusButton2_clicked()
{
	TempOffsetMinusState2=1;
	if (TemperatureOffset2 > -25)
	{
		TemperatureOffset2--;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOffsetPlusButton2_clicked()
{
	TempOffsetPlusState2=1;
	if (TemperatureOffset2 < 25)
	{
		TemperatureOffset2++;
		//STDregisters.Motor_Speed_Percentage = Speed;
	}
}

void on_TempOffsetOKButton2_clicked()
{
	Set_TempOffset_Value2 = TemperatureOffset2;
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(TempOffset2);
}

void on_TempOffsetCancelButton2_clicked()
{
	gtk_widget_hide(CalibrationBlurWindow);
	gtk_widget_hide(TempOffset2);
}


//-------------------------------------------------------Password Page--------------------------------------------------------//

void on_SettingsButton_clicked()
{
	gtk_widget_show(PasswordWindow);
	usleep(10000);
	gtk_widget_hide(HomeWindow);
	usleep(10000);
}

void on_BackButton_clicked()
{
	memset(password_in,0,strlen(password_in));
	passwordlen=0;
	
	gtk_widget_show(HomeWindow);
	usleep(10000);
	gtk_widget_hide(PasswordWindow);
	usleep(10000);
}

void on_Num1_clicked()
{
	if (passwordlen < 4)
	{
		NumState1=1;
		char ch = '1';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num2_clicked()
{
	if (passwordlen < 4)
	{
		NumState2=1;
		char ch = '2';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num3_clicked()
{
	if (passwordlen < 4)
	{
		NumState3=1;
		char ch = '3';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num4_clicked()
{
	if (passwordlen < 4)
	{
		NumState4=1;
		char ch = '4';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num5_clicked()
{
	if (passwordlen < 4)
	{
		NumState5=1;
		char ch = '5';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num6_clicked()
{
	if (passwordlen < 4)
	{
		NumState6=1;
		char ch = '6';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num7_clicked()
{
	if (passwordlen < 4)
	{
		NumState7=1;
		char ch = '7';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num8_clicked()
{
	if (passwordlen < 4)
	{
		NumState8=1;
		char ch = '8';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num9_clicked()
{
	if (passwordlen < 4)
	{
		NumState9=1;
		char ch = '9';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_Num0_clicked()
{
	if (passwordlen < 4)
	{
		NumState0=1;
		char ch = '0';
		strncat(password_in, &ch, 1);
		passwordlen++;
	}
}

void on_EnterButton_clicked()
{
	if(!strcmp(password_in,"1234"))
	{
	gtk_widget_show(MenuWindow);
	usleep(10000);
	gtk_widget_hide(PasswordWindow);
	usleep(10000);
	}
	else
	{
		gtk_widget_show(PasswordBlurWindow);
		gtk_widget_show(PwdCheckWindow);
	}
	
	memset(password_in,0,strlen(password_in));
	passwordlen=0;
}

void on_PwdOKButton_clicked()
{
	gtk_widget_hide(PasswordBlurWindow);
	gtk_widget_hide(PwdCheckWindow);
}