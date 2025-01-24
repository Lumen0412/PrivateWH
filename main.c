//SD231TJ»ðÔÖ±¨¾¯¿ØÖÆÆ÷
//655 HZfactor[]={"ºÏ·ÊÒ»·«Ïû·À"}; //"±±¾©Ê¨µºÏû·À" "ºÏ·ÊÒ»·«Ïû·À"
//2010	if(key==KEY_XY)  //TJ_FuWei    (key==KEY_FW)  //this is Reset
//2018	if(key==KEY_FW)  //TJ_XiaoYin ( key==KEY_XY)  //this is XiaoYin
//Boma: 80==0_ÏÂÔØ 40==40_²»±¨CRT&QuYu-GZ 20==20_<92HJ_=00_<130HJ 04==0_ÂýHJGZ 02==0_²»±¨Ö÷±¸µç¹ÊÕÏ
//      01=0_²»±¨Ö±Æô¹ÊÕÏ»Ø´ð57GZ
//HJalarm(16,0,17,193); 17_193:ZDGZ 194:BDGZ 195:SGGZ 196:CRTGZ 197:QuYu
//Í¨Ñ¶ÅäÖÃ:
//1_ÏÂÔØÉÏ´«, 
//2_CRT_RS232, CRTstrbCRTstrb
//3_»ØÂ·1/2, 
//4_´òÓ¡»ú, 
//5_±¸ÓÃ, 
//6_ÇøÓò¼¯ÖÐ»ú_RS485,
//7_×ÜÏßÅÌ_Ö÷¿Ø, ZongXianPan

//SPI_Flash w40  Block(64k) Sector(4k) 2048k=32_Bx64k=(32x16)_Sx4k
//B 0B   1B  2B  3B  4B  5B  6B  7B   8B   9B   10B  11B  12B  13B  14B  15B  16B ---31B
//S 0~15 16~ 32~ 48~ 64~ 80~ 96~ 112~ 128~ 144~ 160~ 176~ 192~ 208~ 224~ 240~ 256~
//  | 64kx=512k HZK_63800h=398k=7_B ||sys+Rec ||LP_D_1~20|         |LDBC|USB |QuYu
//+128S  0S   1S   2S 3S 4S 5S 6S 7S 8S 9S 10S 11S 12S 13 14S 15S
//80000  0000 1000        84000       88000       8C000 8E000 8F000
//       | HJJL 16K     || LDJL16K  ||QiTaJL 16K |JLZZ| DJB  | PBB

//+144S  0S   1S   2S 3S 4S 5S 6S 7S 8S 9S 10S 11S 12S 13S 14S 15S
//90000  0000 1000                                9C000        9F000
//      |MIMA||SYS|                               |ZXP |       |DEV|

//²Á³ýÒ»¸öÉÈÇø 4k,ÉÈÇøµØÖ· 0~127 for w25x40 time>=150ms SPI_Flash_Erase_Sector(u8 sector)   
//²Á³ýÒ»¸ö¿é(16¸öÉÈÇø)16*4k=64k: time>=150ms SPI_Flash_Erase_Block(u8 block)   
//SPI_Flash_Write_NoCheck(u8* Buf,u32 addr,u16 num) 
//SPI_Flash_Read(u8* Buf,u32 addr,u16 num)

//sub1-16   COM3
//ÏµÍ³¾§Õñ18.432MHz£¬1Ê±ÖÓ NOdiv PCON2_CLKS0_2=0  115200,N81
//BIT01x8+BIT01x7+BIT_QS
//T0:re_puls_count; T2:S1_Boat; CCP:int_fama; Uart1:INT_commu
//communicate:
//So_bz: =0_wait,=1_so_e0_ed,=2_so_cmd_ed,=3_so_dat_over, culi_ed=0;
//Loop1-16   COM3   //communicate:
/* 
     down->                      	 |    back<-    
beg 0  1  2   3   4 ---  25 end  	 | beg  00  01     200 201 -- 225  end   (F3:dat_used FB:flashed) 
F0 01 F2							 | F1   L0  a0  -- a199 s0 -- s24  F3    cmd=1 check_alarm 
F0 02 m0   m1   m2  --   m24 F2 	 | F1   L0  a0  -- a199 s0 -- s24  F3    cmd=2 check_moni+statu
F0 03 seg 00 F2   (seg=0~7)          | F1   n0  n1   val      n24  F3    cmd=3 check_analog_25
F0 04 F2		    		 		 | F1   n0  n1   bit	  n24  F3    cmd=4 check_online
F0 05 adr n  F2  (n=00:run =01:stop) | F1   adr n  moni F3				 cmd=5 QiDong check_online
F0 06 F2	  		 				 | F1   F3			                 cmd=6 Reset No_reutrn
F0 09 adr LL AA NAM FLO ZON RM1 RM2 F2	 | F1   F3   					 cmd=9 CengXian_
F0 0A Badr Bdat F2              	 | F1   Qcmd  Qadr  Qdat F3	(En:writeing)cmd=A Write Addr
F0 0B Badr Bdat F2              	 | F1   Qcmd  Qadr  Qdat F3	(En:writeing)cmd=B Write Zengyi
//ÉÏÎ»»ú£º
//·¢CMD=1,·µ»Ø200×Ö½ÚÄ£ÄâÁ¿+25×Ö½Ú±¨¾¯×´Ì¬
//·¢CMD=2 ¼°25×Ö½Ú(200Î»)Æô¶¯ÃüÁî,·µ»Ø200×Ö½ÚÄ£ÄâÁ¿+25×Ö½Ú±¨¾¯×´Ì¬ 
//·¢CMD=3 //¼°1×Ö½Ú¶ÎµØÖ·ÂëD,1×Ö½Ú00,·µ»ØÇ°ÖÜÆÚ25µØÖ·£¨Dx25£©Ä£ÄâÁ¿ 
//·¢CMD=4,//¼°2×Ö½Ú 00 00,·µ»Ø200Î»online×´Ì¬
//·¢CMD=5 //¼°1×Ö½ÚµØÖ·Âë1×Ö½ÚÍ£Ö¹Æô¶¯Âë,Æô¶¯,·µ»Ø1×Ö½ÚµØÖ·Âë1×Ö½ÚÍ£Ö¹Æô¶¯Âë1×Ö½ÚÄ£ÄâÁ¿ 
//·¢CMD=6 ¸´Î»ÃüÁî,²»·µ»Ø 
//·¢CMD=7 ¼°8×Ö½ÚÊý¾ÝÂëµ½²ãÏÔ,·µ»Ø¸ÃµØÖ·ÀàÐÍ×´Ì¬ 
//·¢CMD=a Ð´µØÖ·ÃüÁî+µØÖ·+Êý¾Ý,½« Êý¾Ý ¸Ä³ÉÐÂ µØÖ·¡£
//Èç"Ð´µØÖ·": ·¢ E0 0a adr cmd E2,
//²ãÏÔ¸´Î»·½·¨:Ö÷»ú·¢Æô¶¯µØÖ·201(05=>201) f0 05 c9 05 f2 »ØÂ·°å½«201×ª·¢255µØÖ·(05=>255)
//²ãÏÔ¸´Î»·½·¨:Cengx45F0v->FE C0 00 00 00 00 00 00 
//²ãÏÔalm Pros_HJ->CengX_alarm(1,ll+1,aa)->CengX_txbuf[i][0,11]=0xf0,CXll
//²ãÏÔalm main->Cengx_loop->CengXstrb[i]->(USART3->DR)
//²ãÏÔ ·¢12 byte adjÊý¾Ý
// 0  1   2     3    4    5    6   7   8    9   10   11  
//f0 09 CXadr alm+ll aa  nam  flo zon rom1 rom2 f2 CXll
//fo 09 CXadr alm+ll aa Mh+D Ml+H Min  00   FE  f2 CXll 
//BIT:150+150+100 Q:2000+100 R:800+3500
//  1  2  3  4  5  6  7  8  9  0  1  2  3   Q     R   
//-__-_--__-_--_--_--_--_--_--_--_--_--__-______-___----------_
//  KZQ => ZXP_GB   COM7
//ÏµÍ³¾§Õñ18.432MHz£¬1Ê±ÖÓ NOdiv PCON2_CLKS0_2=0  115200,N81
// 0  1  2  3 ---  17 18 19 --- 33 34  ret 0  1  2      16 17 
//F0 02 Q0 Q1 --- Q15 H0 H1 ---H15 F2 	| F1 A0 A1 --- A15 F3  cmd=2 check_moni+statu
//F0 06 F2    Reset  No_ans
//F0 07 F2    DengJian	  		 				           | F1   F3		                 cmd=6 Reset No_reutrn

//  KZQ => CRT COM2 9600N81
// 0   1    2   3   4    5    6     7     8     9   10   11  
//fd  7f   00  CMD  LL   AA   NAM   FLO   ZON  ROM1	ROM2 fe
//	    KZQNum    1-18 1-200 0-127 0-127 0-127 ff   0-250
//CMD  6FH =¸´Î»¡£ 42H =»ð¾¯¡£ 6DH =Ñ²¼ì¡£ 67H =¹ÊÕÏ¡£69H =¹ÊÕÏ»Ö¸´
//     63H =Æô¶¯¡£ 65H =Í£Ö¹¡£ 61H =·´À¡¡£ 6BH =³·Ïú¡£51H =ÆÁ±Î¡£ 53H=½â³ý

// KZQ => CengX  to sub_status
// 0   1    2      3      4    5    6     7     8     9   10    11  
// f0 09  CX_AA  CMD+dLL dAA  dNAM dFLO  dZON dROM1 dROM2 fe | CX_LL 
//	      1_200 D76+1-16 1-200 0-7f 0-7f 0-7f   00  0-250
//CMD_D7D6  11 =¸´Î»   10 =×Ô¼ì £¨´ËÊ± D5-0 =0 £© 
//          10 =»ð¾¯¡£ 00 =¹ÊÕÏ¡£ 01 =¹ÊÕÏ»Ö¸´  £¨´ËÊ± D5-0 =dLL £© 

//¼¯ÖÐ»ú-->ÇøÓò»ú¡¡
//Í·(0) Éè±¸(1) ÅÌºÅ(2) ÃüÁî(3)    ¼üÖ·(4) ±¸(5) Çø·Ö(6) ±¸(7)±¸(8)±¸(9)Ð£(10) Î²(11)
//0xfd  0x7d    1-15    0x63(Æô¶¯) 1-127   0     00        0      0       0    0xfe
//0xfd  0x7d    1-15    0x65(Í£Ö¹) 1-127   0     00        0      0       0    0xfe
//               0      0x6f(¸´Î»)   0
//               0      0x6e(µÆ¼ì)   0
//               0      0x60(µÇ¼Ç)   0
//0xfd  0x7d    1-15    0x61(·´À¡) 1-127   0     00        0      0       0    0xfe
//0xfd  0x7d    1-15    0x6b(³·Ïú) 1-127   0     00        0      0       0    0xfe
//0xfd  0x7d    1-15    0x6d(Ñ²¼ì) 1-127   0     00        0      0       0    0xfe
//0xfd  0x7d    1-15    0x50(²éÑ¯) 1-127   0     00/ff     0      0       0    0xfe
//ÇøÓò»ú-->¼¯ÖÐ»ú
//0xfd  0x7d    1-15    0x53(Æô¶¯) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x55(Í£Ö¹) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x51(·´À¡) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x5b(³·Ïú) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x57(¹ÊÕÏ) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x59(»Ö¸´) 1-127   0     00/ff     0      0       0    0xfe
//0xfd  0x7d    1-15    0x5e(Î´´ð) 1-127   0     00/ff     0      0       0    0xfe
//ÇøÓò»ú-->¼¯ÖÐ»ú 
//ÇøÓò:Pross_hj->QuYu_alarm(0x42,ll+1,aa,ut6_max)->;QuYu_txbuf[i][0]=alarm
//QYalm_num +=1; main->Usart_loop6()->USART6->DR =QuYustrb[i];
//¼¯ÖÐ:main->Usart_loop6()->if(ut6_jz==0) case ur6_cmd==0x42: HJalarm(1,ur6_num,ur6_ll,ur6_aa);
//CRT:main->Pross_hj if(ut6_jz==0) CRT_alarm(1,ll+1,aa)->CRT_txbuf[i][0]=alarm
//main->CRT_loop() if(CRT_txbuf[i][0]!=0) USART6->DR =CRTstrb[i]

/* Ö±ÆôÅÌÍ¨Ñ¶ Device: EN8F1823E
No  0   1   2   3   4  5  6  7  8  9  10 11 12 13 14   15  16    17    18 
sobuf[] 0   1   2   3  4  5  6  7  8  9  10 11 12 13   14  15    16    17 
-> F0  02  Q01 Q23                                       Q2829 Q3031   F2
<- F1 K01  K23                                           K3031   F3
->QD:Ò»×Ö½ÚÁ½Â·:0000_Õý³£¡¡¡¡¡¡¡¡¡¡¡¡0010_¹ÊÕÏ 0100_Æô¶¯ 
<-FK:Ò»×Ö½ÚÁ½Â·:0000_²»´æÔÚ 0001_ÔÚÏß 0010_·´À¡ 0100_ÊÖÆô  

//SG_GB:ÏÂÔØ7*.007(Len-1=0xfe),ÉÏµçread_sys ¶Áµ½SGGB[] ÔÚHJalarm=>k=127,SGGB[|80+ll aa]
//ÔÚRfsh SGGB[]->djb[][|20 &df] FGon_off
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"
#include "ff.h"	
#include <stdio.h>

#include "usbh_core.h"
#include "usbh_msc_core.h"
#include "usbh_usr.h"     
#include "usbd_desc.h"
#include "usb_conf.h"
#include "flash_if.h"

#define uchar   unsigned char
#define uint    unsigned int

extern void USART1_SendByte(u16 dat);
extern void UART4_SendByte(u16 dat);
//extern void USART1Write(u8* data,u16 len);

/* Private functions ---------------------------------------------------------*/
void Init_All_Periph(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void RTC_Configuration(void);
void ADC_Configuration(void);
void TIM6_Configuration(void);
void Prnt_ini(void);


void USART1_Configuration(void);
void USART2_Configuration(void);
void USART3_Configuration(void);	//Loop 1
void UART4_Configuration(void);		//Print Paper
void USART6_Configuration(void);
void UART7_Configuration(void);	//Loop 2
void CAN1_Configuration(void);

void SDRAM_Configuration(void);
void SDRAM_InitSequence(void);
void IWDG_Configuration(u8 prv ,u16 rlv);
void DMA_transfer(void);
void SPI1_Configuration(void);

void Clear_Sdram_Lcd(void);
void Read_sys(void);
void Rfsh(void);
void Pross_hj(void); 
//void Pross(void); 
void KEYdeal(void);
void Getkb(void);   //cjy keydeal
void Getkey(void);  //cjy Getkey  yuan GetKey
void StartProg(void);
//void RefreshEdit(void); //ÉèÖÃ²ÎÊý¸öÊý£¬ÏÔÊ¾ÌáÊ¾
//void RefreshEdit2(void);
//void RefreshEdit3(void);
//void CheckPass(void);
//void EditProc(void);
//void RefreshPass(void);

//u8 GetKey(void);
u8 Getcode(u8 i);

void Lcd_ini(void);
void LCD_LayerInit(void);//³õÊ¼»¯LTDµÄ²ã²ÎÊý

void ClearS(u16 curv, u16 curh, u16 width, u16 height, u16 Color);//ÇåÆÁ
void ClearXY(u16 curv, u16 curh, u16 height, u16 weight, u16 Color);
void Delay16s(u16 delx,u16 circle);
void Delay1ms(u16 dms);

void LCD_SetBackColor(uint16_t Color);
void LCD_SetTextColor(uint16_t Color);
void LCD_SetColors(uint16_t TextColor, uint16_t BackColor);
void LCD_SetLayer(uint32_t Layerx);
void LCD_SetTransparency(uint8_t transparency);
void LCD_DrawLine(u16 Xpos,u16 Ypos,u16 Length,u8 Direction,u16 fColor);// »­Ò»ÌõÖ±Ïß
void LCD_DrawSquare(u16 x, u16 y,u16 width,u16 high,u16 color);//»­¿ò
void LCD_DrawSquareBox(u16 x, u16 y,u16 width,u16 high,u16 color);

void Up(void);   
void Down(void);
void Left(void);
void Right(void);
void ChaHJ(void);
void ChaLD(void);
void ChaGZ(void);

void Esc(void);  
void ResetC(void);
void ZiJian(void);

void Logon(void);  
void DengjiSD(void);
void DengjiZD(u8 dcon,u8 dll,u8 daa);

void LiandongQT(void);//Áª¶¯ÊÖ¶¯ÆôÍ£
void Geli(void);     
void Area(void);
void Setup(void);	
void EquSetup(void);

void Displine(u8 type,u8 dwn);//ÀúÊ·¼ÇÂ¼
void Displine_JLB(u8 type,u8 dwn);
void DJBcx(u8 dll);
void PBBcx(u8 dwn);  
void GZcx(u8 dwn);   

void Prntjl(u8 alm,u8 area,u8 dll,u8 daa); 
void Printing (u8 cde);


void HJalarm(u8 alm,u8 area,u8 dll,u8 daa);
void Dispram(u8 alm,u8 hlg,u8 exam,u8 area);
void Logit(u8 alm,u8 area,u8 dll,u8 daa);
void Mdfjl(u16 zz,u32 waddr);
void Modify(u8 hf_alm,u8 dll,u8 daa);
void Mdfzz(u32 waddr);

void liandongBC(uchar dll,uchar daa);
void liandongQD(u16 p,u8 fa);


void Dispzf(u8 ncode,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Disphzs(u16 ccode,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Dispsz16(u16 dcode,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Dispzf24(u8 ncode,u16 curv, u16 curh, u16 bColor,u16 fColor);//ÏÔÊ¾24*12×Ö·û
void Disphzs24(u16 ccode,u16 curv,u16 curh,u16 bColor,u16 fColor);//ÏÔÊ¾24*24ºº×Ö
void Dispsz24(u16 dcode,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor);

void Disptime(u8 *p,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor);
void DispTimeUP(void);
void DispszHEX(u8 dcode,u16 curv,u16 curh,u16 bColor,u16 fColor); 
void Dispalm(void);//ÏÔÊ¾±¨¾¯ÏÞ

void DispMenu(u8 mfk8,u8 mfk4);
void Dispmsg(u8 nfk8,u8 nfk4);					
void Dispinput(void);


void RTC_Time_Init(void);
void Get_time(void);
void RTC_Alarm_Set_time(void);
void BKP_Read(void);
void BKP_Write(void);

void Disptu(u8 numb,u16 color,u16 curv,u16 curh);
void Dispstr(u8 begin,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Dispstr16(u8 begin,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Disprom(u32 addr,u8 dll,u8 daa,u16 curv,u16 curh,u16 bColor,u16 fColor);
void Dispsys(void);
void DispFactor(void);

void RS485_up(u8 equ,u8 mac,u8 alm,u8 dll,u8 daa,u8 nam,u8 flo,u8 zon);
void RS485_down(u8 equ,u8 mac,u8 alm,u8 dll,u8 daa,u8 nam,u8 flo,u8 zon);
void HLCX_down(u8 dllcx,u8 daacx,u8 dllalm,u8 daa,u8 nam,u8 flo,u8 zon);
void Utrs(void);  
void Utrsup(void);
void ProssRS485(void);
void ProssQYJ(void);

void Usart_loop(u8 zz,u8 dll);
void Usart_loop3(u8 dll);
void Usart_SendByte(USART_TypeDef * pUSARTx, u8 ch);
void Usart_SendString(USART_TypeDef * pUSARTx, char *str);
void CAN_SetMsg(CanTxMsg *TxMessage);
void Init_RxMes(CanRxMsg *RxMessage);
void Usart_loop2ZXP(u8 dll);			// Zong Xian Pan
void Usart_loop6(void);			//QuYu
void Usart_loop7ZhiQi(u8 dll);			//this  ZhiQi
void Usart_loop7(u8 dll);		//ZXP Loop
void U7_QDHDcmd(u8 cmd,u16 LPllaa); //cmd=3QD 4TZ 6HD 7HDQX
void Cengx_loop(void);
void CRT_loop(void);
void CRT_Fuwei(void);  //0x6f fw
void CRT_alarm(u8 alarm,u8 ll,u8 aa,u8 quyu);
void CengX_alarm(u8 alarm,u8 ll,u8 aa);  //
void QuYu_alarm(u8 alarm,u8 ll,u8 aa,u8 quyu);  //HJ_GZ_HD QD
void QuYu_loop(void);  //0x6d loop
void QuYu_Fuwei(void);  //0x6f fw
void	LianDongQR(void);
void	JianCha(void);
/*ÐÂÔö*/
void JianChaShow(void);																						//¼ì²é½çÃæÏÔÊ¾
void DispCheck_JLB(int dwn);																			//¼ì²é¼ÇÂ¼±í
void CheckCntStat(void);																					//¼ì²éÊýÁ¿Í³¼Æ
void designCntStat(void);																					//Éè¼ÆÊýÁ¿Í³¼Æ
void NormalWorkStatInc(u8 loop,u8 addr);													//Õý³£¹¤×÷Êý×ÔÔö
void BreakDownStatInc(u8 loop,u8 addr);														//¹ÊÕÏÊý×ÔÔöÍ³¼Æ
void ShieldStatInc(u8 loop,u8 addr);															//ÆÁ±ÎÊý×ÔÔöÍ³¼Æ
void ClearArray(void);																						//Í³¼ÆÊý×é³õÊ¼»¯
/*ÐÂÔö½áÊø*/

//void Urev_loop3(void);

void Urev_loop(void);
void Urev_noloop(void);
void Urev_loop_2(void);
void Urev_noloop_2(void);

u8 SPI_FLASH_SendByte(u8 byte);
void XCBC(void);
u8 Getnum(u8 i);     

void SheZhi_MiMa(void);
void SheZhi_ShiZong(void);
void SheZhi_DengJi(void);
void ChaoZuo_Qidong(void);
void ChaoZuo_GunHang(void);
void ChaoZuo_Bei(void);
void DangAn_BaoJing(void);
void DangAn_PingBi(void);
void DangAn_DengJi(void);
void BianCheng_XianShi(void);
void BianCheng_LianDong(void);
void BianCheng_JianPan(void);
void CheShi_MiNi(void);
void CheShi_DaoRu(void);
void CheShi_DaoChu(void);
void ChaoZuo_PingBi(void);
void KZSG_199(void);

void XCerase(u8 erase);//²Á³ý
void XCXSBC(void);    
void XCXSCX(void);   
void XCKRG(void);     
void XCKRGCX(void);   
void XCLDBC(void);    
void XCLDCX(void); 
void XCCXBC(void);   
void XCCXCX(void);    
void Download_CX(void);
void XCZXBC(void);     
void XCZXCX(void);    

void Valset(void);
void Lpdebug(void);//±¨¾¯Öµ²¹³¥²éÑ¯
void Lpdebugset(void);
void RAMTest(void);   //ÄÚ²¿²ÎÊý²éÑ¯
void TXTest(void); 
void XTJL(void);      //´òÓ¡¼ÇÂ¼
void PrinterJL_date(u8 jl);
void PrinterJL_num(u8 jl);
                  
void Analog(void);     
void AnalogCI(void);   //ÏÔÊ¾Å¨¶ÈÖµ

void Download_PC(void);
void Upload_PC(u8 type);
void Set_point(void);

void moni_disp(void);
u16 GBkeyconv(u8 ll,u8 aa);
void GBconv_key(u8 ll,u8 aa);

u8 Ds1302Read(u8 addr);
void Ds1302Write(uchar addr, uchar dat);
void Ds1302Init();
void DS1302_DIR(u8 i);
//--------------------------FatFs---------------------------//
void USB_HOST_MAIN(u8 num);
void Read_USB_file(u8 num);


__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_Core __ALIGN_END ;
USB_OTG_CORE_HANDLE   USB_OTG_Core;
USBH_HOST             USB_Host;

FATFS fs;          /*FatFs ÎÄ¼þÏµÍ³¶ÔÏó */
FIL fil;
FRESULT res_flash; /* ÎÄ¼þ²Ù×÷½á¹û */
 
UINT fnum;

extern u8 usbresfg;   
u16 bsp_delay; 
   
/*---------------------------- Global variables ------------------------------*/
u8 ROWfirst,ROWsecond,Delay16ms,cnt;
u8 intcp,kbcp,kbdat[4],kb_bz,kk_bak;
u16 workw1;
u8 lpflag_hj,lpflag_gz,lpflag_ad,lp1,lprfsh;
u16 tick,tick1s;
u8 key,fk8,fk4,fk4save=0,fk8save=2;		//first F3 no_opreat
u8 MenuNum[8]={3,3,3,3,3,3,0,0};  //F1ÏîÊý~F5ÏîÊý
u8 DigitNum[6][4]={0xff,4,6,6,0xff,6,6,6,0xff,0,0,0,0xff,2,3,2,0xff,2,6,6,0xff,6,4,4};
	//Ã¿Ïî²ÎÊý¸öÊý        SheZi      Chaozuo    DangAn    BianCheng  TiaoShi   PB QD TC
u8 cEdit,cEditbak;  		// =1,2 Éè²ÎÊý 
u8 bProg;  		// =1 ÓÐ F1-5¼ü°´ÏÂ
u8 cPass;  		// 0,1,2 µ±Ç°µÄÃÜÂë¼¶±ð
u8 iEditPointer,iEditMax;
u8 sEdit[16];    //¼üÈë×Ö·û²ÎÊý [3]:1_begin [2]:2 [1]:3 [0]:4_last 1234 
u8 iEditPos[16]; //¼üÈëÎ»ÖÃ

u16 memory_addr;
//const u8 adzx[6]={10,11,12,13,16,17};
	
const u8 bconv[8]={1,2,4,8,0x10,0x20,0x40,0x80};
const u8 Aconv[25]={0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120,128,136,144,152,160,168,176,184,192};

/* ¶¨Òå DMA ´«ÊäÄ¿±ê´æ´¢Æ÷ */
u8 aDST_Buffer[128]; 
																															
CanTxMsg TxMessage;			   //·¢ËÍ»º³åÇø
CanRxMsg RxMessage;				 //½ÓÊÕ»º³åÇø
																
__align(32) u8 downb[131072] __attribute__((at(0xd0200000)));	//Íâ²¿´æ´¢Çø
__align(32) u8 Tfile[5] __attribute__((at(0xd0177000)));

__align(32) u8 Uart3_recBuf[1024] __attribute__((at(0xd0220000)));	//Íâ²¿´æ´¢Çø
u32 Uart3_recZZ;
u8	Uart3_recOK=0,Uart3_LastrecLen,Uart3_LastCmd,Uart3_LastLL;																																

__align(32) u8 Uart2_recBuf[32] __attribute__((at(0xd0220420)));	//Íâ²¿´æ´¢Çø
u8	U2_recZZ;
u8	U2_recOK=0;
u8	U2_LastrecLen =18;
u8	U2_rxKB[16];     //[0-15] to_use_ZXP 
u8	U2_rxKBbak[16];
u8	U2_Lastcmd;
u8	U2_cmdbz[4]={02,02,02,02};			//zxp_QDBZ [xx]=02_QD_HD led; =6_Reset =7_Dengjian
u8	U2_txQD[4][16];
u8	U2_txHD[4][16];

__align(32) u8 Uart7_recBuf[256] __attribute__((at(0xd0220480)));	//Íâ²¿´æ´¢Çø
u32 Uart7_recZZ;
//u8	U7_xz[4][256];     //0:KBon 1:KBcled 2:QDled 4 HDled
u8	U7_txQD[4][16];     //
u8	U7_txHD[4][16];     //
u8	U7_rxKB[4][16];     //[0,1][i] to_use_ZXP [2][i] to_use_ZhiQiPan 
u8  U7_recOK[4];
u8	U7_LastrecLen[4];
u8	U7_Lastcmd[4];
u8	U7_LastLL;
u8	U7_mode;					//=0_ZXP0 =1_ZXP2 =2_ZhiQiPan1. store U7_rxKB[U7_mode][i]
u8	U7_rxKBbak[4][16];     //if (rxKB ^ KBbak)==1 then  rxKB=1_QD; rxKB=0_TZ;
u8	U7_cmdbz[4]={02,02,02,02};			//zxp_QDBZ [xx]=02_QD_HD led; =6_Reset =7_Dengjian


u16	KBnumadr[4][128];     //Key
u8  ur2_subnum,ur2_cmd,ur2_key,ur2_keyb;																																
u8  ut2_subnum,ut2_cmd,ut2_key,ut2_keyb;
u8  ZQP_Cmd57;			//ZQP Error Fa cmd57
u8  ZQP_txbuf[32][3]; //[0]=alm( >80 used,Reset=0),[1]=ll,[2]=aa
u8  LDGBstrb[16]={0xfd,0x7d,0x01,0x6d,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,}; //´®¿Ú2ÏÂ·¢

u8 	Uart6_recBuf[32];  //QuYu_CRT
u32 Uart6_recZZ;  
u8	Uart6_recOK=0;
u8  Uart6_RecDat;  
u8  QuYu_GZ_count;
u8	ut6_num,ut6_max,ut6_jz; //JiZong:µ±Ç° _¡¡×ÜÊý /ÇøÓòºÅ 1:ÇøÓò0¼¯ÖÐ
u8	ut6_cmd,ut6_ll,ut6_aa;
u8	ur6_num,ur6_cmd,ur6_ll,ur6_aa;																																
//u8	ut6_len=13;
u8	QYalm_num=0;
u8  QuYu_txbuf[32][4]; //When_QYJ_is_HJ_andUP,JZJ_is_QD_andDown.[0]=alm,[1]=ll,[2]=aa,[3]=Quyu
u8  QuYustrb[16]={0xfd,0x7a,0x00,0x6d,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,}; //´®¿Ú6ÏÂ·¢
u8	QYdjb[16][200];
u8	QYdjbgz[16][200];
u8  CRT_txbuf[200][4]; //[0]=alm( >80 used,Reset=0),[1]=ll,[2]=aa,[3]=quyu
u8  CRTstrb[16]={0xfd,0x7f,0x00,0x6d,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,}; //´®¿Ú7ÏÂ·¢

u8  CengX_RecDat;  //CengX
u8  CengX_txbuf[20][12]; 
u8  CengXstrb[16]={0xf0,0x09,0x00,0x01,0x01,0x01,0x01,0x01,0x34,0x12,0xf2,0x00}; 
//                  tou cmd CXadr alm+ll aa nam  flo  zon  rom1 rom2 wei  CXll 
///////////////////////////////////////////
u8 hjflag; //»ð¾¯±êÖ¾
u8 gzflag; //¹ÊÕÏ±êÖ¾
u8 ldflag; //Áª¶¯±êÖ¾
u8 zdgz;   //Ö÷µç¹ÊÕÏ±êÖ¾
u8 bdgz;   //±¸µç¹ÊÕÏ±êÖ¾
u8 Menukey;
u8 MenuBF; 
u8 MenuBX; 
u8 enter;  
u8 MenuC;  //Ö÷²Ëµ¥±êÊ¶
u8 MenuF; 
u8 MenuN;  
u8 machine;//±¾»úµØÖ·
u8 Tlcd;		// =5 off_LCD
u16 counter;
u16 sumz;   
////////////////////////////////////////////
u8 knum;
u8 shu[12];
u8 com;
u8 Top;
u8 zero;      //¿Õ±êÖ¾
//////////////////////////////////////////
u16 pass[3];//µÇÂ¼ÃÜÂë
u8 passf;  
u8 password;
u8 gzjc;  
u8 DownL;  //1-ÏÂÔØ
u8 llpp;   
u8 printer;
u8 sgjc;   
u8 bdjc;   
u8 CRTxj;  
u8 date;   //ÉÏµçÈÕÆÚ
//////////////////////////////////////////////////
u8 opfg;      //²Ù×÷±êÊ¶
u8 hjjlfg;   
u8 ldjlfg;   
u8 qtjlfg;   
u8 jlend;		//=0 begin =1 JL_Disp_end    
u8 djfg;     
u8 glfg;     
u8 ldfg;      
u8 djbfg;     
u8 pbbfg;    
u8 gzbfg;     
u8 hjcxfg;    
u8 gzcxfg;    
u8 ldcxfg;   
u8 cifg;     
u16 iwdgtim;  //¶ÀÁ¢¿´ÃÅ¹·¼ÆÊ±
////////////////////////////////////////////////
u16 hjjlzz;   //»ð¾¯¼ÇÂ¼Ö¸Õë
u16 hjjltrue;  
u16 tjlzz;    //ÆäËü¼ÇÂ¼Ö¸Õë
u16 tjltrue;
u16 ldjlzz;    //Áª¶¯¼ÇÂ¼Ö¸Õë
u16 ldjltrue;
u8 hjzz;     
u8 gzzz;     
u8 ldzz;     
u8 LedEQU;    
u8 LedJZ;    
u8 djbloop,djbaddr;   //µÇ¼Ç±í ²éÑ¯ »ØÂ·ºÅ µØÖ·ºÅ
u16 djbcount ;   //µÇ¼Ç±í ²éÑ¯ ¼ÆÊý
u8	pbbloop,pbbaddr;   //ÆÁ±Î±í ²éÑ¯ LL AA
u16	pbbcount;		// ÆÁ±Î±í ¼ÆÊý
u8 xspbs;    
u8 xhzz;      //Ñ²»·Ö¸Õë
/////////////////////////////////////////////////
u8 Tled=0;		//Secont ":"
u8 autofg;    //¶¯×÷Ä£Ê½
u8 manual_ld; //Áª¶¯Æô¶¯ÈË¹¤È·ÈÏ
u8 ldhds;
u8 xslds;
u8 xshds;  
u8 ldqdfg;   
u8 qdhdfg;    
u8 XJ2;      
u8 XJ6;     
//u8 ldqdfg;        
u32 ID;
//////////////////////////////////////////////
u8 LEDbuffer[3];//Ö¸Ê¾µÆ×´Ì¬ out
u8 ZQbuffer;    //ÒôÏì¼ÌµçÆ÷×´Ì¬ out
u8 Boma;   			//²¦Âëin
u8 Powerdat;		//Ö÷±¸µç×´Ì¬in
u8 ctime[6];  
u8 lcdrf;      
u8 txcsfg;   //Í¨Ñ¶²âÊÔ±êÖ¾  					  
//////////////////////////////////////////////
u8 inpzz2; 
u8 outpzz2;  
u8 prlong2; 
u8 psum2;    
u8 tsum;  
u8 wtsum; 
u8 inpzz6;  
u8 outpzz6; 
u8 prlong6;  
u8 psum6;   
u8 tsumup;   
u8 wtsumup;  
u8 inpzzhd; 
u8 inpzzhdl; 
u8 trswait;
u8 trswaitup;
u32 dwlong;  
u32 dwpack; 
///////////////////////////////////////////////
u32 midlong;
u8 inpzz;
u8 prlong;  //½ÓÊÕÊý¾Ý°ü³¤£¬×î¶à12×Ö½Ú
u8 inpzz3;	//½ÓÊÕÊý¾Ý°üÖ¸Õë
//u8 outpzz3;
u8 prlong3;  //½ÓÊÕÊý¾Ý°ü³¤£¬×î¶à225×Ö½Ú
u8 inpzz7;
u8 outpzz7;
u8 prlong7;
u8 psum7;
u8 inpzzhd7;  //»Ø´ðÖ¸Õë
u8 qyjnum;
u8 Spalarm,SpalmOK=0;  //=1 199Éù¹â±¨¾¯  QD, SpalmOK=0 can_key_SG
u8 sggz;    
u8 lphd;     
///////////////////////////////////////////////
u8 fire_lo,fire_hi;   //fire at 85-92 or 85-130
u8 lptestfg;
u8 hjcycle;
u8 gzcycle;  
u8 dscycle;  //»ØÂ·Í¨Ñ¶ÖÜÆÚ
u8 cicycle;  
u8 lpcxsum;  
u8 lpcxfg;  
u8 lpcxzz;   
u8 lxnum;   
u8 zxnum;   

u8 zxztb[8];	
u8 cxztb[16];	
u16 zxq[6];
u16 zxg[6];

u8 lpxz[8][27];   //»ØÂ·1
u8 lpxz_2[8][27]; //»ØÂ·2
u8 lpcxb[40][10]; //»ØÂ·²ãÏÔ±í

u8 lpxz_val[18][202];   //»ØÂ·1
u8 lpxz_bit[18][202]; //»ØÂ·bit_ALM
u8 lpxq_bz[18]; 	//»ØÂ·_QDBZ [xx]=01_Loop;=2_»ØÂ·ÓÐÆô¶¯; =6_Reset

u8 lpxz_zt[18];   //»ØÂ· short test
u8 lpgzjs[18];

u8 haveHJ;

u8 lpzz;   //»ØÂ·Ö¸Õë
u8 lpfresh;//ÃüÁî±êÖ¾
u8 lpsection;
//u8 Cmdsig[8][50];//»ØÂ·µãµÆ


u8 pack1[12][32]; //´®¿Ú1½ÓÊÕ
u8 pack2[8][12];  //´®¿Ú2½ÓÊÕ
u8 pack3[12][32]; //´®¿Ú3½ÓÊÕ
u8 pack6[8][12];  //´®¿Ú6½ÓÊÕ
u8 pack7[8][12];  //´®¿Ú7½ÓÊÕ
u8 strbup[32][12]; //´®¿Ú6ÉÏ´«
u8 strbuf[32][12]; //´®¿Ú2ÏÂ·¢
u8 strblp[100][6]; //yuan´®¿Ú3ÏÂ·¢
u8 lpstrb[16][29]; //´®¿Ú3ÏÂ·¢.[loop][0:°ü³¤ 1:E0 2:cmd 3:xx]
//u8 lpstrb7[4][35]; //´®¿Ú7_GBloopÏÂ·¢.[loop][0:°ü³¤ 1:F0 2:cmd 3:xx]

u8 xz[18][202];     //Ì½²âÆ÷Êý¾Ý
u8 djb[22][202];    //»ØÂ·µÇ¼Ç±í(RAMÇø) 0-15 Ì½²âÆ÷ 16 Ö±ÆôÅÌ
          //7 ,6, 5 ,4, 3, 2,  1,    0,
          //dj,pb,qd,      GZ  HJ/HD I/O
u8 namb[20][202];	// 1-63=HJ 64-124=QD HD 125=XHS
u8 hjjs[18][202];   
u8 gzjs[18][202];   
u8 unasw[18][202];
u8 CI[18][200];      

u8 Disp_pb[64][8]; 
u8 Disp_gz[256][8]; // 0   1   2   3   4 5 6 7
u8 Disp_hj[128][8]; //alm dll daa area   time
u8 Disp_ld[128][8];

u8 cxb[500];			//LiandongBC=>LiandongQD



u8 DISPBIT[4]={0x00,0x40,0x80,0xc0};
u8 KeyTable[32]={0x15,0x00,0x08,0x1b,
	               0x14,0x01,0x09,0x1c,
	               0x17,0x02,0x0b,0x1d,
	               0x11,0x03,0x0c,0x1e,
	               0x12,0x04,0x0d,0x16,
	               0x18,0x05,0x0f,0x20,
	               0x0a,0x06,0x1a,0x21,
	               0x13,0x07,0x1f,0x0e           
				        };							

u8 CDMenu[]={"ÉèÖÃµÇÂ¼Ê±ÖÓµÇ¼Ç²Ù×÷Æô¶¯ÆÁ±Î´òÓ¡µµ°¸¼ÇÂ¼ÆÁ±ÎµÇ¼Ç±à³ÌÏÔÊ¾Áª¶¯¼üÅÌµ÷ÊÔÄ£Äâµ¼Èëµ¼³ö"};
u16 CDpozX[4]={775,750,725,700};		//800-n-24n, n=1234   
u16 CDpozY[5]={132,190,248,306,364};//{190,248,306,364,422}; 	//480-10n-48n,n=54321
u8 CDMenuMsg0[]={"´Ë´¦¿Õ°×ÏÔÊ¾ÊäÈëËÄÎ»ÃÜÂëÒ»Ê±¼ä¶þÈÕÆÚÒ»µÇ¶þÇå»ØµØ             "};
u8 CDMenuMsg1[]={"´Ë´¦¿Õ°×ÏÔÊ¾Ò»Æô¶þÍ£»ØµØÒ»ÆÁ¶þÏû»ØµØÊäÈëÁùÎ»Êý×Ö             "};
u8 CDMenuMsg2[]={"´Ë´¦¿Õ°×ÏÔÊ¾°´È·ÈÏ¼ü¿ªÊ¼°´È·ÈÏ¼ü¿ªÊ¼°´È·ÈÏ¼ü¿ªÊ¼             "};
u8 CDMenuMsg3[]={"´Ë´¦¿Õ°×ÏÔÊ¾ÊäÈë»ØÂ·È·ÈÏÊäÈë±àºÅÈ·ÈÏÊäÈë¼üºÅÈ·ÈÏ             "};
u8 CDMenuMsg4[]={"´Ë´¦¿Õ°×ÏÔÊ¾ÊäÈë»ØÂ·È·ÈÏÊäÈëÁùÎ»Êý×ÖÊäÈëÁùÎ»Êý×Ö             "};
u8 CDMenuMsg5[]={"´Ë´¦¿Õ°×ÏÔÊ¾Ò»ÆÁ¶þÏû»ØµØÊäÈëËÄÎ»Êý×ÖÊäÈëËÄÎ»Êý×Ö             "}; //PB FW ZJ XX
u16 CDMsgx=800-96;    		//ÌáÊ¾ÐÅÏ¢xÎ»ÖÃ
u16 CDMsgy=480-50-48*5-102;	//ÌáÊ¾ÐÅÏ¢yÎ»ÖÃ
u16 InMsgx=800-96-16;			//ÊäÈëÊý×ÖÐÅÏ¢xÎ»ÖÃ
u16 InMsgy=480-50-48*5-84;	//ÊäÈëÊý×ÖÐÅÏ¢yÎ»ÖÃ
u16 nob_x=26,alm_x=26+33,nam_x=26+33+49,zon_x=26+33+49+98;
u16 flo_x=26+33+49+98+74,rom_x=26+33+49+98+74+28;
u16 tim_x=26+33+49+98+74+28+194;     //26, 33, 49, 74, 28, 194,112
u16 adr_x=26+33+49+98+74+28+194+112; //tou nob alm zon flo rom adr
u16 left_x=26,righ_x=672;
u16 HJtop_y=27,HJlow_y=135,LDtop_y=163,LDlow_y=271,GZtop_y=299,GZlow_y=407;
u16 TimeUP_x=800-96, TimeUP_y=26;
u16 TiShi_x=800-96,TiShi_y=480-34;
u8 HZfactor[]={"Ìì½òÐÂÑÇÏû·À"};    //"±±¾©Ê¨µºÏû·À" "ºÏ·ÊÒ»·«Ïû·À""Ê¨µºÏû·À°²ÐÅ""Ìì½òÐÂÑÇÏû·À""Õã½­°®µÂÏû·À"
u16 sys_1x=18,sys_2x=490,sys_3x=700,sys_1y=480-34,sys_2y=480-17;
u8 HZsys[]={"°æ±¾ »ð¾¯ Æô¶¯ ·´À¡ µÇ¼Ç ¹ÊÕÏ ÆÁ±Î ÉÏÏß µÇÂ¼ µçÔ´ Áª¶¯ Â·9"};
u8 POWmenu[]=("Õý³£Ö÷µç±¸µçÊÖ¶¯×Ô¶¯µçÔ´¹ÊÕÏ");
u16 LP_Num[18]; //Loop OnLine num
u16 VZHJ=0,VZQD=0,VZHD=0,VZGZ=0,VZDJ=0,VZPB=0,VZSX=0,VZLP=0;
u8 VZQDHD=0,VZZQDJ=0,VZZQ=0; //QD!=HD
u8 Kyxbz=0;  //=1 ÒÑ°´ÏûÒô
u8 Lp_max=16;  //MiMa Set max Loop Num
u8 moniLL=0,moniBZ=0;
u8 dispZON=0;	//=01_HJ =02_LD =03_GZ
/*ÐÂÔö*/
u8 chaType=0; //=0_µ±Ç°±¨¾¯ =1_µµ°¸HJLDGZ =2_ÆÁ±Î =3_µÇ¼Ç  =4_¼ì²é =5_Ä£Äâ
u8 ChaHJpg=0,ChaLDpg=0,ChaGZpg=0; //Cha_JLB_pgx5=zz
int Checkpg=0;
u8 hjzzbak,ldzzbak,gzzzbak;
/*ÐÂÔö½áÊø*/
u8 HZMenu[]={"1ÉèÖÃ2²Ù×÷3µµ°¸"};
u8 HZMenuF1[]={"4Ê±ÖÓ5ÃÜÂë6µÇ¼Ç7ÊÖµÇ8Éè±¸"};
u8 HZMenuF2[]={"4Æô¶¯5ÁªÍø6¼ì²â"};
u8 HZMenuF3[]={"4µÇ¼Ç5ÆÁ±Î6¹ÊÕÏ"};
u8 EMenu[]={"´òÓ¡»ú±¸µç¼ì²â¼üÅÌ×ÓÕ¾CRT"};
u8 TMenu[]={"Äê/ÔÂ/ÈÕÊ±:·Ö"};
u8 DLMenu[]={"ÃÜÂë/ÐÞ¸Ä³É¹¦ÍË³ö"};
u8 DJMenu[]={"µØÖ·1µÇ0É¾Íê³ÉÂúµãÉ¾»ØÂ·"};
u8 TSMenu[]={"¼ì²â1¿ª0¹Ø"};//¼ì²â
u8 GLMenu[]={"µØÖ·1ÆÁ0·Å"};//ÆÁ±Î
u8 LDMenu[]={"µØÖ·1Æô0Í£"};//Áª¶¯

u8 XCfile[]={"1Á¬½ÓÓÅÅÌ2Çý¶¯Õý³£3¹ÒÔØ³É¹¦4¹ÒÔØÊ§°Ü5µ¯³öÓÅÅÌ7¶ÁÈ¡ÎÄ¼þ8¡¡¡¡¡¡¡¡9ÏµÍ³±à³Ì"};//ÏÖ³¡±à³Ì
u8 XCMenu[]={"1ÏÔÊ¾±à³Ì2Áª¶¯±à³Ì3Éè±¸Ãû³Æ4²ãÏÔ±à³Ì5×ÜÏß±à³Ì7Éù¹â¹ã²¥8ÇøÓò±à³Ì9ÏµÍ³±à³Ì"};//ÏÖ³¡±à³Ì
u8 XCBCMenu[]={"6±à³Ì7²éÑ¯8Çå¿Õ9ÉÏ´«"}; //ÏÖ³¡±à³Ì×Ó
u8 XCCSMenu[]={"6µãµÆ7×´Ì¬8Í¨Ñ¶9´òÓ¡"};  //Õû»ú²âÊÔ×Ó
u8 XCXTMenu[]={"6Ä£ÄâÁ¿7Å¨¶È"};//ÏµÍ³¼ì²é×Ó
u8 XCCXMenu[]={"6±à³Ì7²éÑ¯8Çå¿Õ9ÏÂÔØ"};    //Í¨Ñ¶±à³Ì×Ó

u8 XSMenu[]={"µØÖ· Éè±¸ Â¥²ã ·ÖÇø ×¢ÊÍ ±àºÅ"};//ÏÖ³¡ÏÔÊ¾±à³Ì
u8 BMMenu[]={"»ØÂ·µØÖ· 1¶Á0Ð´ ±àÂëµØÖ·"};//±àÂë
u8 FLMenu[]={"ÎÄ¼þÒÑ±£´æÐ´Èë´íÎó"};
u8 HMenu[]={"±¨¾¯µØÖ·Éè±¸Ãû³Æ·ÖÇøÂ¥²ã×¢ÊÍÊ±¼äÐòºÅ"};
/*ÐÂÔö*/
int TypeMenu[] = {0,1,2,3,4,5,6,7,8,9,10,11,12};
int TypeCnt = sizeof(TypeMenu)/sizeof(TypeMenu[0]);
int design_Cnt[13] = {0};
int design_Cnt_n = sizeof(design_Cnt)/sizeof(design_Cnt[0]);
struct check{
	char *TypeName;
	u8 DesignCnt;
	u8 NormalCnt;
	u8 BreakdownCnt;
	u8 ShieldCnt;
};
u8 typeNameMenu[] = {"¸ÐÑÌÌ½²â¸ÐÎÂÌ½²âÊÖ¶¯±¨¾¯Ïû»ðË¨Å¥ÆäËû±¨¾¯Ïû·À±Ã  ÅçÁÜ±Ã  ÅÅÑÌ»ú  ËÍ·ç»ú  µçÌÝÆÈ½µÓ¦¼±¹ã²¥·À»ð¾íÁ±ÆäËûÊä³ö"};
u8 CheckMenu[] = {"ÀàÐÍÉè¼ÆÊýÁ¿Õý³£¹¤×÷Êý¹ÊÕÏÊýÆÁ±ÎÊý"};
u16 type_x = 26,
		DesignCnt_x = 26+33+49+98+50,
		NormalCnt_x = 		26+33+49+98+50+120,
		BreakdownCnt_x = 	26+33+49+98+50+120+140,
		ShieldCnt_x = 26+33+49+98+50+120+140+100;

int SmokeDetectionCnt = 0;		//¸ÐÑÌÌ½²âÊýÁ¿
int ManualAlarmCnt = 0;				//ÊÖ¶¯±¨¾¯ÊýÁ¿
int OtherAlarmCnt = 0;				//ÆäËû±¨¾¯ÊýÁ¿
int OtherOutputCnt = 0;				//ÆäËûÊä³ö
int DesignCntArray[13] = {0};																													//Éè¼ÆÊýÁ¿Êý×é
int DesignCntArray_n = sizeof(DesignCntArray)/sizeof(DesignCntArray[0]);							//Éè¼ÆÊýÁ¿Êý×é´óÐ¡
int NormalWorkCntArray[13] = {0};																											//Õý³£¹¤×÷ÊýÁ¿Êý×é
int NormalWorkCntArray_n = sizeof(NormalWorkCntArray)/sizeof(NormalWorkCntArray[0]);	//Õý³£¹¤×÷ÊýÁ¿Êý×é´óÐ¡
int BreakdownCntArray[13] = {0};																											//¹ÊÕÏÊýÁ¿Êý×é
int BreakdownCntArray_n = sizeof(BreakdownCntArray)/sizeof(BreakdownCntArray[0]);			//¹ÊÕÏÊýÁ¿Êý×é´óÐ¡
int ShieldCntArray[13] = {0};																													//¹ÊÕÏÊýÁ¿Êý×é
int ShieldCntArray_n = sizeof(ShieldCntArray)/sizeof(ShieldCntArray[0]);							//¹ÊÕÏÊýÁ¿Êý×é´óÐ¡
/*ÐÂÔö½áÊø*/
u8 Version[]={"V1.0"};
u8 *dstr;

u8 READ_RTC_ADDR[6] = {0x8d, 0x89, 0x87, 0x85, 0x83, 0x81};  
u8 WRITE_RTC_ADDR[6] = {0x8c, 0x88, 0x86, 0x84, 0x82, 0x80};//??????  ???????
u8 FI2C_time[6];
u8 FI2C_stime[6];

u16	SGGB[512];     				//Sengguang -> Guangbo
u8	bzSGGB =0;
u8	Tsggb=0;

u8 BZ_CHECK=0,BZ_LDQR=0;
//               1      2     3     4     5     6  
u16  xaa[17]={0x0104,0x0140,0x0004,0x0040,0x0304,0x0340,0x0204,0x0240,0x0504,0x0540,0x0404,0x0440,0x0604,0x0640,0x0504,0x0540};
//           1 2 3 4     5     6  
u8  yaa[17]={4,3,2,1,8,7,6,5,12,11,10,9,16,15,14,13};
/*---------------------------- Global variables ------------------------------*/
 
/* ÓÃÓÚ´æ´¢µ±Ç°×ÖÌåÑÕÉ«ºÍ×ÖÌå±³¾°ÑÕÉ«µÄ±äÁ¿*/
 static uint16_t CurrentTextColor = 0x001F;
 //static uint16_t CurrentTextColor = 0x0000;
 static uint16_t CurrentBackColor = 0xFFFF;
/* ÓÃÓÚ´æ´¢²ã¶ÔÓ¦µÄÏÔ´æ¿Õ¼ä ºÍ µ±Ç°Ñ¡ÔñµÄ²ã*/
 static uint32_t CurrentFrameBuffer = LCD_FRAME_BUFFER;
 static uint32_t CurrentLayer = LCD_BACKGROUND_LAYER;



void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

/* USART1   Tx (PA.09)  Rx (PA.10)*/ 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
  	GPIO_Init(GPIOA, &GPIO_InitStructure);  
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
  	GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
 			
/*USART2  Tx (PA.02)  Rx (PA.03)*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

/*USART3 Tx (PB.10)  Rx (PB.11)*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;     
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);		
		
/*UART4  */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
						
/*USART6  Tx (PG.14)  Rx (PG.9)*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;     
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
  	GPIO_Init(GPIOG, &GPIO_InitStructure); 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);	
		
/*UART7 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOF, &GPIO_InitStructure); 	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
  	GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_UART7);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_UART7);			
				
/*CAN1  Tx (PB.09)  Rx (PB.08)*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_CAN1);
		
/*CAN2 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6, GPIO_AF_CAN2); 
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_CAN2);
		
/*SPI1 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;    //SCK         
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	   //MISO       
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    //MOSI         
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	  //CS           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &GPIO_InitStructure);		
/*SPI2 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource3,GPIO_AF_SPI2); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	           
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource2,GPIO_AF_SPI2); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;             
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	             
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &GPIO_InitStructure);				
/*MCO1 */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
/*SDRAM*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;      
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;    
 
 /*AÐÐÁÐµØÖ·ÐÅºÅÏß*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    // A0-PF0           
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_FMC);    	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                         
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource2,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource3,GPIO_AF_FMC);	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource4,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource5,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource12,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource13,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource14,GPIO_AF_FMC); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;               
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource15,GPIO_AF_FMC); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource0,GPIO_AF_FMC); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource1,GPIO_AF_FMC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  		// A12-PG2             
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource2,GPIO_AF_FMC);
  
	/*BAµØÖ·ÐÅºÅÏß*/	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;      // BA0-PG4          
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource4,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;       // BA1-PG5         
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource5,GPIO_AF_FMC);
  /*DQÊý¾ÝÐÅºÅÏß*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;       // D0-PD14         
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;               
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;               
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FMC); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;               
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;               
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FMC);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;               
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FMC);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FMC);	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;               
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;              
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FMC);	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;              
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;               
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FMC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    // D15-PD10          
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FMC);

  /*¿ØÖÆÐÅºÅÏß*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;       //CS MEM2      
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;        //WE        
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource0,GPIO_AF_FMC);	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;      //RAS       
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource11,GPIO_AF_FMC);     
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;     //CAS         
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource15,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;      //CLK        
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource8,GPIO_AF_FMC);    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;       //CKE        
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOH,GPIO_PinSource7,GPIO_AF_FMC);  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;      //DQM0         
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_FMC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;      //DQM1         
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_FMC);
	
/*LCD*/
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 

 /* Êý¾ÝÏßR1~R5  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //R1-PJ0        
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource0, GPIO_AF_LTDC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource1, GPIO_AF_LTDC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;          
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource2, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;        
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource3, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;    //R5-PJ4         
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource4, GPIO_AF_LTDC);
	
	/* Êý¾ÝÏßG0~G5  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;     //G0-PJ7     
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource7, GPIO_AF_LTDC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource8, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource9, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource10, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;         
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource11, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //G5-PK0      
  GPIO_Init(GPIOK, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource0, GPIO_AF_LTDC);
	
	/* Êý¾ÝÏßB1~B5  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;    //B1-J13      
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource13, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource14, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;          
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOJ, GPIO_PinSource15, GPIO_AF_LTDC);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;          
  GPIO_Init(GPIOK, &GPIO_InitStructure); 
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource3, GPIO_AF_LTDC);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //B5-PK4       
  GPIO_Init(GPIOK, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOK, GPIO_PinSource4, GPIO_AF_LTDC);
	
	 /*¿ØÖÆÐÅºÅÏß*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;            
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource7 ,GPIO_AF_LTDC); //LCD_clk_PG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_LTDC); //LCD_Hsync_PC6
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;          
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4 ,GPIO_AF_LTDC); //LCD_Vsync_PA4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource10 ,GPIO_AF_LTDC); //LCD_DE_PF10 CJY

/*i2c*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4 |GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  //GPIO_PinAFConfig(GPIOH,GPIO_PinSource4, GPIO_AF_I2C2); 
  //GPIO_PinAFConfig(GPIOH,GPIO_PinSource5, GPIO_AF_I2C2);
	
/*GPIOI*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOI, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOI, &GPIO_InitStructure);
	
/*GPIOH  cjy redefine*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOH, &GPIO_InitStructure);
		
	
/*µØÖ·¶Ë¿Ú*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

/*µôµç¼ì²â*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/*RS485·½Ïò*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOJ, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void NVIC_Configuration(void)
{ 
  	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM  
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
    #endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
			
//USART1 Interrupt
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	            
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       	 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	       
    NVIC_Init(&NVIC_InitStructure);
		
//USART2 Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
    NVIC_Init(&NVIC_InitStructure);
		
//USART3 Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
    NVIC_Init(&NVIC_InitStructure);	
		
//USART6 Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
    NVIC_Init(&NVIC_InitStructure);
		
//UART7 Interrupt
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
    NVIC_Init(&NVIC_InitStructure);		
			
//CAN1 Interrupt		
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);		
}

void RCC_Configuration(void)
{
	HSE_SetSysClock(8,360,2,7);     
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 |RCC_APB2Periph_USART6 |RCC_APB2Periph_ADC3  
	                      |RCC_APB2Periph_SPI1   |RCC_APB2Periph_SPI4  |RCC_APB2Periph_LTDC , ENABLE );
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  |RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3 | RCC_APB2Periph_TIM1 , ENABLE );
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM2 |RCC_APB1Periph_USART2 |RCC_APB1Periph_USART3 |RCC_APB1Periph_UART7 
	                      |RCC_APB1Periph_UART4| RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2   | RCC_APB1Periph_SPI2, ENABLE );
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1|RCC_AHB1Periph_DMA2 |RCC_AHB1Periph_DMA2D,ENABLE);//´ò¿ªDMA1,DMA2µÄÊ±ÖÓ
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
                        |RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF
	                      |RCC_AHB1Periph_GPIOG |RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI
	                      |RCC_AHB1Periph_GPIOJ |RCC_AHB1Periph_GPIOK , ENABLE);
	
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FMC, ENABLE); 
}


void TIM6_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  /* ÀÛ¼ÆTIM_Period¸öÖÜÆÚºó²úÉúÒ»¸ö¸üÐÂ»òÕßÖÐ¶Ï*/
  TIM_TimeBaseStructure.TIM_Period = 499;  
  TIM_TimeBaseStructure.TIM_Prescaler = 8999;
  // ³õÊ¼»¯¶¨Ê±Æ÷ TIMx, x[2,3,4,5]
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM6, TIM_FLAG_Update);   
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); 
 } 

void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 115200 ;			                 
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	         
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			         
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			         
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; 
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART1, ENABLE);                    
}

void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 115200 ;                 
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	  
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			 
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART2, ENABLE);                     
}

void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 115200  ;               
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	  
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			  
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); 
	USART_Cmd(USART3, ENABLE);                     
}

void UART4_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 9600  ;               
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	  
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			 
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4, ENABLE);                     
}

void USART6_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 9600  ;                
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	  
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			  
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);  
	USART_Cmd(USART6, ENABLE);                     
}


void UART7_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate            = 115200  ;               
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;	 
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;			 
	USART_InitStructure.USART_Parity              = USART_Parity_No ;			  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART7, &USART_InitStructure);	
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);  
	USART_Cmd(UART7, ENABLE);                     
}


void CAN_Mode_Config(void)
 {
 CAN_InitTypeDef CAN_InitStructure;

 CAN_DeInit(CAN1);
 CAN_StructInit(&CAN_InitStructure);

 /*CAN µ¥Ôª³õÊ¼»¯*/
 CAN_InitStructure.CAN_TTCM=DISABLE; 
 CAN_InitStructure.CAN_ABOM=ENABLE;  
 CAN_InitStructure.CAN_AWUM=ENABLE;  
 CAN_InitStructure.CAN_NART=DISABLE;
 CAN_InitStructure.CAN_RFLM=DISABLE;

 CAN_InitStructure.CAN_TXFP=DISABLE; 
 CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
 CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;

 CAN_InitStructure.CAN_BS1=CAN_BS1_5tq; //BS1¶ÎÕ¼ÓÃ 5 ¸öÊ±¼äµ¥Ôª
 CAN_InitStructure.CAN_BS2=CAN_BS2_3tq; //BS2¶ÎÕ¼ÓÃ 3 ¸öÊ±¼äµ¥Ôª
 CAN_InitStructure.CAN_Prescaler =5;
 CAN_Init(CAN1, &CAN_InitStructure);
 }

 
void CAN_Filter_Config(void)
 {

  CAN_FilterInitTypeDef CAN_FilterInitStructure;

 /*CAN É¸Ñ¡Æ÷³õÊ¼»¯*/
  CAN_FilterInitStructure.CAN_FilterNumber=0; 
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;    
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   
  CAN_FilterInitStructure.CAN_FilterIdHigh= ((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;
  CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;

  CAN_FilterInit(&CAN_FilterInitStructure);
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
 }


void CAN1_Configuration(void)
{
  CAN_Mode_Config();
  CAN_Filter_Config();  
}

void Lcd_ini(void)
{ 
  LTDC_InitTypeDef    LTDC_InitStruct;
	//¸ü¸ÄLCD»­ÖÊ
	RCC_PLLSAIConfig(384,7,4);
	RCC_LTDCCLKDivConfig(RCC_PLLSAIDivR_Div16);
  /* Ê¹ÄÜ PLLSAI Ê±ÖÓ */
  RCC_PLLSAICmd(ENABLE);
  while(RCC_GetFlagStatus(RCC_FLAG_PLLSAIRDY) == RESET){}

  /*ÐÅºÅ¼«ÐÔÅäÖÃ*/
  LTDC_InitStruct.LTDC_HSPolarity = LTDC_HSPolarity_AL;     
  LTDC_InitStruct.LTDC_VSPolarity = LTDC_VSPolarity_AL;    
  LTDC_InitStruct.LTDC_DEPolarity = LTDC_DEPolarity_AL;      
  LTDC_InitStruct.LTDC_PCPolarity = LTDC_PCPolarity_IPC;   
  
  /* ÅäÖÃLCD±³¾°ÑÕÉ« */                   
  LTDC_InitStruct.LTDC_BackgroundRedValue = 0;            
  LTDC_InitStruct.LTDC_BackgroundGreenValue = 0;          
  LTDC_InitStruct.LTDC_BackgroundBlueValue = 0;    
 
  /* Ê±¼ä²ÎÊýÅäÖÃ */  
  LTDC_InitStruct.LTDC_HorizontalSync =HSW-1;
  LTDC_InitStruct.LTDC_VerticalSync = VSW-1;
  LTDC_InitStruct.LTDC_AccumulatedHBP =HSW+HBP-1;  
  LTDC_InitStruct.LTDC_AccumulatedVBP = VSW+VBP-1; 
  LTDC_InitStruct.LTDC_AccumulatedActiveW = HSW+HBP+LCD_PIXEL_WIDTH-1;
  LTDC_InitStruct.LTDC_AccumulatedActiveH = VSW+VBP+LCD_PIXEL_HEIGHT-1;
  LTDC_InitStruct.LTDC_TotalWidth =HSW+ HBP+LCD_PIXEL_WIDTH  + HFP-1; 
  LTDC_InitStruct.LTDC_TotalHeigh =VSW+ VBP+LCD_PIXEL_HEIGHT + VFP-1; 	
  LTDC_Init(&LTDC_InitStruct);
  LTDC_Cmd(ENABLE);
}  

void LCD_LayerInit(void)
{
  LTDC_Layer_InitTypeDef LTDC_Layer_InitStruct; 
  
  /* ²ã´°¿ÚÅäÖÃ */
	LTDC_Layer_InitStruct.LTDC_HorizontalStart = HBP + HSW;
	LTDC_Layer_InitStruct.LTDC_HorizontalStop = HSW+HBP+LCD_PIXEL_WIDTH-1;
	LTDC_Layer_InitStruct.LTDC_VerticalStart =  VBP + VSW;
	LTDC_Layer_InitStruct.LTDC_VerticalStop = VSW+VBP+LCD_PIXEL_HEIGHT-1;	
  /* ÏñËØ¸ñÊ½ÅäÖÃ*/
	LTDC_Layer_InitStruct.LTDC_PixelFormat =LTDC_Pixelformat_RGB565;
  LTDC_Layer_InitStruct.LTDC_ConstantAlpha = 255; 
  /* Ä¬ÈÏ±³¾°ÑÕÉ«£¬¸ÃÑÕÉ«ÔÚ¶¨ÒåµÄ²ã´°¿ÚÍâ»òÔÚ²ã½ûÖ¹Ê±Ê¹ÓÃ¡£ */          
  LTDC_Layer_InitStruct.LTDC_DefaultColorBlue = 0;        
  LTDC_Layer_InitStruct.LTDC_DefaultColorGreen = 0;       
  LTDC_Layer_InitStruct.LTDC_DefaultColorRed = 0; 
  LTDC_Layer_InitStruct.LTDC_DefaultColorAlpha = 0;     
  LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_CA;    
  LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_CA;
	LTDC_Layer_InitStruct.LTDC_CFBLineLength = ((LCD_PIXEL_WIDTH * 2) + 3); 
  /* ´ÓÄ³ÐÐµÄÆðÊ¼Î»ÖÃµ½ÏÂÒ»ÐÐÆðÊ¼Î»ÖÃ´¦µÄÏñËØÔöÁ¿ */ 
  LTDC_Layer_InitStruct.LTDC_CFBPitch = (LCD_PIXEL_WIDTH * 2);  
  LTDC_Layer_InitStruct.LTDC_CFBLineNumber = LCD_PIXEL_HEIGHT;   
  LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER;
  LTDC_LayerInit(LTDC_Layer1, &LTDC_Layer_InitStruct);	
  LTDC_Layer_InitStruct.LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
  /* ÅäÖÃ±¾²ãµÄÏÔ´æÊ×µØÖ·£¬ÕâÀïÅäÖÃËü½ô°¤ÔÚµÚ1²ãµÄºóÃæ*/     
	LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER + BUFFER_OFFSET;
	LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_CA;    
  LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_CA;
  LTDC_LayerInit(LTDC_Layer2, &LTDC_Layer_InitStruct);  
  LTDC_ReloadConfig(LTDC_IMReload);
  LTDC_LayerCmd(LTDC_Layer2, ENABLE);  
  LTDC_ReloadConfig(LTDC_IMReload);
  LTDC_DitherCmd(ENABLE); 
}


void SDRAM_Configuration(void)
{
  FMC_SDRAMInitTypeDef  FMC_SDRAMInitStructure;
  FMC_SDRAMTimingInitTypeDef  FMC_SDRAMTimingInitStructure;  
 /* SDRAMÊ±Ðò½á¹¹Ìå£¬¸ù¾ÝSDRAM²ÎÊý±íÅäÖÃ----------------*/	
  FMC_SDRAMTimingInitStructure.FMC_LoadToActiveDelay    = 2;      
  FMC_SDRAMTimingInitStructure.FMC_ExitSelfRefreshDelay = 7;
  FMC_SDRAMTimingInitStructure.FMC_SelfRefreshTime      = 5;       
  FMC_SDRAMTimingInitStructure.FMC_RowCycleDelay        = 10;         
  FMC_SDRAMTimingInitStructure.FMC_WriteRecoveryTime    = 2;      
  FMC_SDRAMTimingInitStructure.FMC_RPDelay              = 2;                
  FMC_SDRAMTimingInitStructure.FMC_RCDDelay             = 2;
	
	/*Ñ¡Ôñ´æ´¢ÇøÓò*/
  FMC_SDRAMInitStructure.FMC_Bank = FMC_Bank2_SDRAM;                          
  FMC_SDRAMInitStructure.FMC_ColumnBitsNumber = FMC_ColumnBits_Number_8b;     
  FMC_SDRAMInitStructure.FMC_RowBitsNumber = FMC_RowBits_Number_12b;          
  FMC_SDRAMInitStructure.FMC_SDMemoryDataWidth = FMC_SDMemory_Width_16b ;   
  FMC_SDRAMInitStructure.FMC_InternalBankNumber = FMC_InternalBank_Number_4;
  FMC_SDRAMInitStructure.FMC_CASLatency = FMC_CAS_Latency_3 ;               
  FMC_SDRAMInitStructure.FMC_WriteProtection = FMC_Write_Protection_Disable;
	FMC_SDRAMInitStructure.FMC_SDClockPeriod = FMC_SDClock_Period_3;
  FMC_SDRAMInitStructure.FMC_ReadBurst = FMC_Read_Burst_Disable;   
	/* ¶ÁÑÓ³ÙÅäÖÃ */
  FMC_SDRAMInitStructure.FMC_ReadPipeDelay = FMC_ReadPipe_Delay_1;        
  FMC_SDRAMInitStructure.FMC_SDRAMTimingStruct = &FMC_SDRAMTimingInitStructure;
  FMC_SDRAMInit(&FMC_SDRAMInitStructure);  
  SDRAM_InitSequence();  
}


void SDRAM_InitSequence(void)
{
  FMC_SDRAMCommandTypeDef FMC_SDRAMCommandStructure;
  uint32_t tmpr = 0;

  FMC_SDRAMCommandStructure.FMC_CommandMode = FMC_Command_Mode_CLK_Enabled;
  FMC_SDRAMCommandStructure.FMC_CommandTarget = FMC_Command_Target_bank2;   
  FMC_SDRAMCommandStructure.FMC_AutoRefreshNumber = 1;                    
  FMC_SDRAMCommandStructure.FMC_ModeRegisterDefinition = 0;
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM , FMC_FLAG_Busy) != RESET){}
  FMC_SDRAMCmdConfig(&FMC_SDRAMCommandStructure);    
  Delay16s(50,50);  
/*¶ÔËùÓÐµÄbankÔ¤³äµç */ 
  FMC_SDRAMCommandStructure.FMC_CommandMode = FMC_Command_Mode_PALL;     
  FMC_SDRAMCommandStructure.FMC_CommandTarget = FMC_Command_Target_bank2;
  FMC_SDRAMCommandStructure.FMC_AutoRefreshNumber = 1;
  FMC_SDRAMCommandStructure.FMC_ModeRegisterDefinition = 0;
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){}
  FMC_SDRAMCmdConfig(&FMC_SDRAMCommandStructure); 
/*×Ô¶¯Ë¢ÐÂ */   
  FMC_SDRAMCommandStructure.FMC_CommandMode = FMC_Command_Mode_AutoRefresh;  
  FMC_SDRAMCommandStructure.FMC_CommandTarget = FMC_Command_Target_bank2;    
	FMC_SDRAMCommandStructure.FMC_AutoRefreshNumber = 4;	
  FMC_SDRAMCommandStructure.FMC_ModeRegisterDefinition = 0;
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){}
  FMC_SDRAMCmdConfig(&FMC_SDRAMCommandStructure);
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){}
  FMC_SDRAMCmdConfig(&FMC_SDRAMCommandStructure);	
  tmpr = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |SDRAM_MODEREG_CAS_LATENCY_3   |
                   SDRAM_MODEREG_OPERATING_MODE_STANDARD |SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
 
  /*ÉèÖÃSDRAM¼Ä´æÆ÷ */
  FMC_SDRAMCommandStructure.FMC_CommandMode = FMC_Command_Mode_LoadMode;   
  FMC_SDRAMCommandStructure.FMC_CommandTarget = FMC_Command_Target_bank2;  
  FMC_SDRAMCommandStructure.FMC_AutoRefreshNumber = 1;                    
  FMC_SDRAMCommandStructure.FMC_ModeRegisterDefinition = tmpr; 
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){}
  FMC_SDRAMCmdConfig(&FMC_SDRAMCommandStructure);
	FMC_SetRefreshCount(448);	
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){}
}



void SPI1_Configuration(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_FLASH_CS_HIGH();     
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;         //Ë«ÏßÈ«Ë«¹¤
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                              //Ö÷»úÄ£Ê½
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                         
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                               
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                             
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;       
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                         //ÏÈ·¢ËÍMSB
  SPI_InitStructure.SPI_CRCPolynomial = 7;                                   //²»Ð£Ñé
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE); 
}


void Prnt_ini(void)
{
 Printing(0x1b);Printing(0x40);              //´òÓ¡»úÈí¼þ³õÊ¼»¯
 Printing(0x1b);Printing(0x31);Printing(0x4);//ÉèÖÃÐÐ¼ä¾àÎª4
}

void RTC_Configuration(void)
{
  RTC_InitTypeDef RTC_InitStructure;
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);	//Ê¹ÄÜPWRÊ±ÖÓ
  //PWR_BackupAccessCmd(ENABLE);
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) { }
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  RCC_RTCCLKCmd(ENABLE);
  RTC_WaitForSynchro();
 //=====================³õÊ¼»¯Í¬²½/Òì²½Ô¤·ÖÆµÆ÷µÄÖµ======================//
	RTC_InitStructure.RTC_AsynchPrediv = 0x01;           //1-40KHz
	RTC_InitStructure.RTC_SynchPrediv = 0x4E1F;          //19999-40KHz
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	//=====================³õÊ¼»¯Í¬²½/Òì²½Ô¤·ÖÆµÆ÷µÄÖµ======================//
	//RTC_InitStructure.RTC_AsynchPrediv = 127;
	//RTC_InitStructure.RTC_SynchPrediv = 255;
	//RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
}

void IWDG_Configuration(u8 prv ,u16 rlv)
{
  
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);	 
  IWDG_SetPrescaler(prv);	                      
  IWDG_SetReload(rlv);                           
  IWDG_ReloadCounter();	                        
}


void CAN_RX_IRQHandler(void)
{
	/*´ÓÓÊÏäÖÐ¶Á³ö±¨ÎÄ*/
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	 tick++;
//	 if(tick%2==0)Ur485_1=1;
//	 else         Ur485_1=0;  CJY gai
	if((RxMessage.ExtId==0x1314) && (RxMessage.IDE==CAN_ID_EXT) && (RxMessage.DLC==8) ) //½ÓÊÕ³É¹¦  
	  {
  	}
	else
	 {
	   PAout(0)=0; 					   //½ÓÊÕÊ§°Ü
	 }
}

void USART1_IRQHandler(void)  //ÏÂÔØ
{
	u8 ch;u32 i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	{
		ch=USART1->DR;
		if(txcsfg==1){downb[dwpack]=ch;dwpack++;return;}  	
		if(DownL==2)//RS232ÏÂÔØ
		{	if(ch==0xfe)
			{	downb[dwlong]=ch;
				if(downb[1]==2)
				{	if(downb[2]==0)	{if(dwlong==(dwpack+6)){DownL=0xff;midlong=dwlong;}}
					else  					{DownL=0xff;midlong=dwlong;}
					dwlong=0;							    
				}
				else
				{	if(dwlong==(dwpack+6))DownL=0xff;
					else{	for(i=0;i<=65536;i++)downb[i]=0;dwpack=0;}
					dwlong=0;
				}
			} 											 
			if(dwlong>0&&dwlong<=65536)
			{	downb[dwlong]=ch;dwlong++;
				if(dwlong==4){if(downb[3]!=0x4c){for(i=0;i<10;i++)downb[i]=0;dwlong=0;dwpack=0;}}
				if(dwlong==6) dwpack=downb[4]*256+downb[5];//°üÄÚÈÝ³¤ yuan *100
      }
			if(ch==0xfd){if(dwlong==0){downb[dwlong]=ch;dwlong++;}}//°üÍ·
		}
		else//Õý³£Êý¾Ý
		{	if(prlong==26)
			{	lpxz_2[0][prlong]=ch;
				if((ch==0xf3)||(ch==0xf7)||(ch==0xf1))inpzz++;
				prlong=0;
				return;
			}
			if(prlong>0&&prlong<26){lpxz_2[0][prlong]=ch;prlong++;}								
      if(prlong==0){if(ch==0xf1){lpxz_2[0][prlong]=ch;prlong++;}}	
		}
	}	 
}


void USART2_IRQHandler(void) //jianpan_zhiqi
{
	u8 ch,i;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
 	{ 	
		ch=USART2->DR; 
		if(U2_recZZ==0) 
		{	if(ch==LoopS) 			//0xfd
			{	Uart2_recBuf[0]=ch; U2_recZZ++;
			}
		}
		else 
		{	Uart2_recBuf[U2_recZZ]=ch; U2_recZZ++;
			if(U2_recZZ>=U2_LastrecLen)
			{	U2_recZZ=0;
				if(ch==LoopR)		//0xfe
				{
//Test_13=1;
					U2_recOK=1;
					for(i=0;i<16;i++)
					{ U2_rxKB[i]=Uart2_recBuf[i+1];}
//Test_13=0;				
				}
			}
		}
	}	
		
}


void USART6_IRQHandler(void)  //QuYu_CRT
{
	u8 ch;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{	ch=USART6->DR;
		if(Uart6_recZZ==0) 
		{	if(ch==0xfd) //0xfd
			{	Uart6_recBuf[0]=ch; Uart6_recZZ++;
//Test_13=1;
			}
//			if(ch==0x85) 
//			{	Uart6_recBuf[0]=ch; Uart6_recOK=1;  //=0x85
//Test_13=1;
//			}
		}
		else 
		{	Uart6_recBuf[Uart6_recZZ]=ch; Uart6_recZZ++;
//USART1_SendByte(ch);			
			if(Uart6_recZZ>=13)
			{	Uart6_recZZ=0;
				if(ch==0xfe)
				{	if(Uart6_recBuf[1]==0x7a) Uart6_recOK=2; //QuYu
					if(Uart6_recBuf[1]==0x7f) Uart6_recOK=3; //CRT
				}
//Test_13=0;				
			}
		}

	} 	 
}


void USART3_IRQHandler(void)  //´®¿Ú»ØÂ·Êý¾Ý(µÚ1/2»ØÂ·)
{
	u8 ch;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {	
		ch=USART3->DR;
		if(Uart3_recZZ==0) 
		{	if(ch==LoopS)
			{	Uart3_recBuf[0]=ch; Uart3_recZZ++;
//Test_13=1;
			}
		}
		else 
		{	Uart3_recBuf[Uart3_recZZ]=ch; Uart3_recZZ++;
			if(Uart3_recZZ>=Uart3_LastrecLen)
			{	Uart3_recZZ=0;
				if(ch==LoopR) {	Uart3_recOK=1; }
//Test_13=0;				
			}
		}
	}		
}


void UART7_IRQHandler(void)  //ZXP_»ØÂ·Êý¾Ý
{
	u8 ch,i;
	if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)
  {	Test_12=1;
		ch=UART7->DR;
//USART1_SendByte(ch);
		if(Uart7_recZZ==0) 
		{	if(ch==LoopS)
			{	Uart7_recBuf[0]=ch; Uart7_recZZ++; }
		}
		else 
		{	Uart7_recBuf[Uart7_recZZ]=ch; Uart7_recZZ++;
			if(Uart7_recZZ>=U7_LastrecLen[U7_mode])
			{	Uart7_recZZ=0;
				if(ch==LoopR) 
				{
//					i=Uart7_recBuf[1];
//					if(i==0x12)
					U7_recOK[U7_mode]=2;	//store at U7_rxKB[2][i]
//					if((i==2)||(i==6)||(i==7)) Uart7_recOK=1;
					Test_12=0;
					for(i=0;i<16;i++)
					{ U7_rxKB[U7_mode][i]=Uart7_recBuf[i+1];}
				}
			}		//Test_13=0;				
		}
	}		
}


//50ms¶¨Ê±ÖÐ¶Ï
void TIM6_DAC_IRQHandler (void)
{
	{ 
		kbcp=intcp&0x03;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);		 
		GPIOI->ODR=(ZQbuffer&0x3f) | DISPBIT[kbcp]; 
		ZQZJ_on();Delay16s(26,1); 
		Ledscan_off();
		
		GPIOI->ODR = LEDbuffer[1]; 
		Ledscan1_on();Delay16s(26,1);  //20,20_is13us; 16,1_is0.61us
		Ledscan_off();

		GPIOI->ODR=LEDbuffer[2];
		Ledscan2_on();Delay16s(26,1); 
		Ledscan_off();

		PowerS_on();Delay16s(26,1);
		workw1=GPIOI->IDR; Powerdat=workw1>>8;
		Ledscan_off();

	  Boma_on();Delay16s(26,1);
		workw1=GPIOI->IDR; Boma=workw1>>8;
		Ledscan_off();

		Keyscan_on();Delay16s(26,1);    
	  workw1= ~(GPIOI->IDR); kbdat[kbcp]=workw1>>8;		//»Ø¶Á°´¼ü
		Ledscan_off();

		if(kbcp==3) 
		{	kb_bz|=kbdat[0];kb_bz|=kbdat[1];kb_bz|=kbdat[2];kb_bz|=kbdat[3];
//USART1_SendByte(kb_bz);
		}

		intcp+=1; if(intcp>=96) intcp=0;

		tick1s++;cnt++;
		if(cnt==2){cnt=0;lpfresh++;}//100msÏÂ·¢Ò»´Î»ØÂ·ÃüÁîÊý¾Ý
		bsp_delay++;if(bsp_delay>800)bsp_delay=0;//USB¶¨Ê±
	} 
}

void Getkb(void)
{ 
  //if(iwdgtim>200){iwdgtim=0;IWDG_ReloadCounter();}//10SÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
	u8 i,kk,kbuf[4];
  if(kb_bz !=0)
  {  for(i=0;i<4;i++) { kbuf[i]=kbdat[i]; }
     switch(kbuf[0]) {
       case 0x01: kk=KEY_LDFS; break; //kk1
       case 0x02: kk=KEY_F1; break;
       case 0x04: kk=KEY_FW; break;
       case 0x08: kk=KEY_XY; break;
       case 0x10: kk=KEY_SG; break;
       case 0x20: kk=KEY_F5; break;
       case 0x40: kk=KEY_HJ; break;
       case 0x80: kk=KEY_LD; break;
     }
     switch(kbuf[1]) {
       case 0x01: kk=KEY_F4; break;
       case 0x02: kk=KEY_1 ; break;
       case 0x04: kk=KEY_2 ; break;
       case 0x08: kk=KEY_3 ; break;
       case 0x10: kk=KEY_XX; break;
       case 0x20: kk=KEY_PB; break;
       case 0x40: kk=KEY_UP; break;
       case 0x80: kk=KEY_QR; break;
     }
     switch(kbuf[2]) {
       case 0x01: kk=KEY_F3; break;
       case 0x02: kk=KEY_4 ; break;
       case 0x04: kk=KEY_5 ; break;
       case 0x08: kk=KEY_6 ; break;
       case 0x10: kk=KEY_0 ; break;   
       case 0x20: kk=KEY_LF; break;
       case 0x40: kk=KEY_GZ; break;
       case 0x80: kk=KEY_RI; break;
     }
     switch(kbuf[3]) {
       case 0x01: kk=KEY_F2; break;
       case 0x02: kk=KEY_7 ; break;
       case 0x04: kk=KEY_8 ; break;
       case 0x08: kk=KEY_9 ; break;
       case 0x10: kk=KEY_ZJ; break;
       case 0x20: kk=KEY_QD; break;
       case 0x40: kk=KEY_DW; break;
       case 0x80: kk=KEY_TC; break;
     }
//		if(kk==kk_bak) { key=kk; //USART1_SendByte(key); }
//     else {  kk_bak=kk; key=0; }
		key=kk;
    for(i=0;i<4;i++) { kbdat[i]=0; }
    kb_bz=0;
  }  
 }
void DS1302_DIR(u8 i)    //i=1_out =0_in
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	if(i==1)
	{ GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	}
	else  { GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; }
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOH, &GPIO_InitStructure);
}
/*------------------------------Ö÷·½·¨------------------------------*/
int main(void){
	u8 j;
	Init_All_Periph();
	Clear_Sdram_Lcd();																		//Çå¿ÕSDRAM
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);	//Ê¹ÄÜPWRÊ±ÖÓ
  PWR_BackupAccessCmd(ENABLE);

	if(RTC_ReadBackupRegister(RTC_BKP_DR1)!=0xa5a5){
		RTC_Configuration();																//RTC ÅäÖÃ
		RTC_Time_Init();
	}
	else{
		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);  
		//PWR_BackupAccessCmd(ENABLE);   
		RTC_WaitForSynchro(); 
		BKP_Read();
		//for(j=0;j<5;j++) { USART1_SendByte(ctime[j]);Delay16s(20,20);}

		RTC_Configuration();																//RTC ÅäÖÃ
	}

	Read_sys();
	Logit(37,0,0,0);																			//¹Ø»ú 
	Logit(36,0,0,0);																			//¿ª»ú 
	ROWfirst=ROWsecond=0;
	djbloop=1;djbaddr=1; djbcount=0;
	cicycle=1;lxnum=1;zxnum=1;  
	passf=0;
	Disptu(15,Yellow,0,0);																//³õÊ¼»¯
	//while(DownL==2){ClearS(50,0,800,24,Magenta);Download_PC();}//ÏÂÔØ
	TIM_Cmd(TIM6, ENABLE);
	//while(Tini<3){if(lpflag_ad==1){lpflag=0;Tini++;}}
	Esc();
	if(xspbs>0) LedPB_on();//ÆÁ±ÎÖ¸Ê¾µÆÁÁ 	 
	//IWDG_Enable(); 
	fk8save=0; fk8=1; fk4save=3; fk4=0;							// ÏÔÊ¾²Ëµ¥
//	DispMenu(0,0);
	DispMenu(1,0);	DispMenu(2,0);	DispMenu(3,0);	DispMenu(4,0);
	for(j=0;j<7;j++) sEdit[j]=35; 	 // ******
	bProg=1;
	
	for(j=0;j<16;j++) lpxq_bz[j]=1;  //sub basic out cmd
	Ur485_1=0;	Ur485_2=0;  //Sou
  dscycle=33;
	USART1_SendByte(0xcc);
	Test_12=1;Test_13=1;
	Delay16s(16,1);
	Test_12=0;
	Test_13=0;
	KLCD_on();	
	for(j=0;j<20;j++) CengX_txbuf[j][11]=20;
  Ds1302Init();
//USART1_SendByte(0xcb);

	Delay1ms(4000);  //4s   8000
//	ResetC();
//hjcycle=3;	gzcycle=3;  //test_factore CJY
	
//djb[16][1]=djb[16][2]=djb[16][3]=djb[16][4]=0x81;
	//ÐÂÔö
	//¼ì²é×ÜÊýÍ³¼Æ
	designCntStat();
	while(1)
	{
		if(lpfresh>=1)
		{	lpfresh=0;tick=0;
			inpzz3=0;inpzz=0;				 	
			if(lpzz<=7)	{	Cengx_loop(); Usart_loop3(lpzz);}   //Current Loop 0-7
			if((lpzz>=8)&&(lpzz<=15))												 //Current Loop 8-16
			{	if(lpzz==8){ Usart_loop6();}  // QuYu_JiZong
				Cengx_loop(); Usart_loop3(lpzz);
			} 
			if(lpzz==16)	{ Usart_loop7ZhiQi(2);	}//Usart_loop6();} // QuYu_JiZong 
			if(lpzz==17)	{	CRT_loop();Usart_loop2ZXP(0);  }  				  
			lpzz++; if(lpzz>=18){lpzz=0;}
		}
		if(tick1s>=20) //1s_1 Sech bbbbbbbbbbbbbbbbbbbbbbb
		{	tick1s=0;
			Rfsh();
			//ÐÞ¸Ä 1.22 Ä£ÄâÁ¿½çÃæ£¬ÏµÍ³ÌõË¢ÐÂ
			Pross_hj();
			if((moniLL!=0)&&(moniBZ==1))
			{	moni_disp(); }
//j=Ds1302Read(0x81);USART1_SendByte(j);j=Ds1302Read(0x83);USART1_SendByte(j);j=Ds1302Read(0x85);USART1_SendByte(j);j=Ds1302Read(0x87);
//USART1_SendByte(j);j=Ds1302Read(0x89);USART1_SendByte(j);j=Ds1302Read(0x8B);USART1_SendByte(j);j=Ds1302Read(0x8d);USART1_SendByte(j);
		}	
//USART1_SendByte(autofg);Delay16s(20,20);
//		if(autofg==1)	{SetLdbz();}
		Getkb();
		Getkey();
  }	//while(1) end 
}


void KEYdeal(void)
{ 
   //if(iwdgtim>200){iwdgtim=0;IWDG_ReloadCounter();}//10SÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
	u8 i,kk;//kbuf[4];
  if(kb_bz !=0)
  {  //for(i=0;i<4;i++) { kbuf[i]=kbdat[i]; }
		 key=KeyTable[kk];// USART1_SendByte(kk);
		 Delay16s(26,300);// USART1_SendByte(key);
     for(i=0;i<4;i++) { kbdat[i]=0; }
     kb_bz=0;
  }   
}

void Getkey(void)
{ u8 key_val,i;
	if(key!=0)
	{ 
//		USART1_SendByte(key);
		KLCD_on();
		if(((key&0xf0)==0x70)||(key==KEY_PB))		   // F1 --F5  + Zhi_Jie
		{	switch (key)
			{	case KEY_F1: fk8=0;break; 
				case KEY_F2: fk8=1;break; 
				case KEY_F3: fk8=2;break; 
				case KEY_F4: fk8=3;break; 
				case KEY_F5: fk8=4;break;
//			case KEY_PB: {fk8=5;fk4=1; bProg=1;USART1_SendByte(0x34);}break;
			}
//		if(fk8<=4)                    // F1 --F5 
			{	if (fk8!=fk8save)		//ÐÂ°´Fn
				{ bProg=1;
					fk4=0; fk4save=0;
				}
				else				//ÖØ¸´°´Fn
				{ fk4save=fk4;fk4++;
					if (fk4>MenuNum[fk8]) fk4=0;
					Dispmsg(fk8,fk4);					
				}
			}	
			cEdit=DigitNum[fk8][fk4];				//²ÎÊý¸öÊý
			cEditbak=cEdit;	
			Dispmsg(fk8,fk4);								//ÏÔÊ¾ÌáÊ¾		
			DispMenu(fk8,fk4);							//ÏÔÊ¾²Ëµ¥
			Dispinput();			 					//ÏÔÊ¾ÊäÈëÊý
			for(i=0;i<7;i++) sEdit[i]=35;  // ******
			fk8save=fk8;
//	USART1_SendByte(fk8save);

		}
		if((key&0xf0)==0x10)			         // num 0-9
		{	switch (key)
			{	case KEY_0:key_val=0x30; break;
				case KEY_1:key_val=0x31; break;
				case KEY_2:key_val=0x32; break;
				case KEY_3:key_val=0x33; break;
				case KEY_4:key_val=0x34; break;
				case KEY_5:key_val=0x35; break;
				case KEY_6:key_val=0x36; break;
				case KEY_7:key_val=0x37; break;
				case KEY_8:key_val=0x38; break;
				case KEY_9:key_val=0x39; break;
			}
			if (bProg!=0)	// F1~5 °´,in²ÎÊý
			{ if(cEdit!=0xff)
				{	if(cEdit==0) cEdit=DigitNum[fk8][fk4];
					cEdit--; 
					sEdit[cEdit]=key_val;
					Dispinput();			 					//ÏÔÊ¾ÊäÈëÊý
				}
			}
		}
		if(key==KEY_QR)  //			case KEY_QR:           /*    ?? key   _/ key */
		{	if ((bProg!=0) && (cEdit==0) && (fk4>0) )
			{	StartProg();
//				dstr=TMenu;Dispstr(0,8,407,380,White,Black); //´òÓ¡ÌáÊ¾
//				Dispzf('!',427,572,White,Black);
//				USART1_SendByte(0xc0);
			}	
//			else { //USART1_SendByte(0xc1); }
		}
		if(key==KEY_FW)   //TJ:KEY_XY  WH:KEY_FW  //this is FW Reset
		{	if(cPass>=1)
			{	//USART1_SendByte(0xa0);
				ResetC();
			}
		}
		if(key==KEY_ZJ)  //			Zi_Jian
		{
     	if(cPass>=1)
			{	//USART1_SendByte(0xa3);
				ZiJian();
			}		
//     USB_HOST_MAIN();			
		}
		if(key==KEY_XY)  //TJ:KEY_FW WH:KEY_XY  //this is	XiaoYin		
		{	//USART1_SendByte(0xa1);
			KZYX_off(); //if((ZQbuffer & 0x01)==0x01) elseKZYX_on();
			LedXY_on();
			Kyxbz =1;
		}
		if(key==KEY_SG)  //			SengGuang_mianban
		{	if(cPass>=1)
			{	if(SpalmOK==0)
				{	//USART1_SendByte(0xa4);
					if(Spalarm==0)
					{	Spalarm=1; SpalmOK=1;  LedSG_on(); 	KZSG_199();Delay1ms(2000);SpalmOK=0;}
					else
					{ Spalarm=0; SpalmOK=1;  LedSG_off(); KZSG_199();Delay1ms(2000);SpalmOK=0;}
				}
			}
		}
		if(key==KEY_LDFS)  //			LianDongQueRen_mianban
		{	//USART1_SendByte(0xa5);
//			Up();
		}
		if(key==KEY_UP)  //			UP
		{	
			//USART1_SendByte(0xd1);
			Up();
		}
		if(key==KEY_DW)  //			Down
		{	
			//USART1_SendByte(0xd2);
			Down();
		}
		if(key==KEY_RI)  //			Righ
		{	//USART1_SendByte(0xd3);
			Right();
		}
		if(key==KEY_LF)  //			Left
		{	//USART1_SendByte(0xd4);
			Left();
		}
		if(key==KEY_HJ)  //			ChaHJ
		{	//USART1_SendByte(0xd5);
			ChaHJ();
		}
		if(key==KEY_LD)  //			ChaLD
		{	//USART1_SendByte(0xd6);
			ChaLD();
		}
		if(key==KEY_GZ)  //			ChaGZ
		{	//USART1_SendByte(0xd7);
			ChaGZ();
		}
		if(key==KEY_PB)  //			Ô­KEY_PB£½¡·¸Ä¡¡Í£Ö¹
		{	fk8=1; fk4=2; bProg=1;
			cEdit=DigitNum[fk8][fk4];				//????
			cEditbak=cEdit;	
			Dispmsg(fk8,fk4);								//????		
			DispMenu(fk8,fk4);							//????
			Dispinput();			 					//?????
			for(i=0;i<7;i++) sEdit[i]=35;  // ******
			fk8save=fk8;
//USART1_SendByte(0xd8);
//			ChaoZuo_PingBi();
		}
		if(key==KEY_TC)  //			KEY_TC
		{	
			/*ÐÂÔö*/
			if(chaType!=0)  //No µ±Ç° is µµ°¸ ÆÁ±Î µÇ¼Ç ¼ì²é Ä£Äâ
			{	//USART1_SendByte(0xd9);
				Esc();
				fk8save=0;	fk4save=3;	DispMenu(0,0);							//ÏÔÊ¾²Ëµ¥
				DispMenu(1,0);	DispMenu(2,0);	DispMenu(3,0);	DispMenu(4,0);
				chaType=0; BZ_CHECK=0;	moniBZ=0;	//ÐÂÔö
			}
			/*ÐÂÔö½áÊø*/
		}
		if(key==KEY_XX)  //			Ô­KEY_XX¡¡¸Ä¡¡´òÓ¡
		{	//USART1_SendByte(0xda);
//			ChaoZou_XX();
		}
		if(key==KEY_QD)  //			KEY_QD
		{ fk8=1; fk4=1; bProg=1;
			cEdit=DigitNum[fk8][fk4];				//????
			cEditbak=cEdit;	
			Dispmsg(fk8,fk4);								//????		
			DispMenu(fk8,fk4);							//????
			Dispinput();			 					//?????
			for(i=0;i<7;i++) sEdit[i]=35;  // ******
			fk8save=fk8;
//USART1_SendByte(0xdb);
//			if(key==KEY_QR) {	ChaoZuo_Qidong();	}
		}

	}		//if(key!=0) end
	key=0;
	if((K_CHECK==0))  		//¼ì²é			
	{	
		BZ_CHECK=1;
		//ÐÂÔö
		JianCha();
		
	}
	if((K_LDQR==0)&&(BZ_LDQR==0))  		//Áª¶¯È·ÈÏ			
	{	
		BZ_LDQR=1;
		LianDongQR();
	}
}

void DispMenu(u8 mfk8,u8 mfk4)
{	u16 pozx,pozy,bcol,fcol; u8 i;
	if(fk8!=fk8save){
		dstr=CDMenu; 
		pozx=CDpozX[0]; pozy=CDpozY[fk8save];
		Dispstr(fk8save*16,1,pozy,pozx,Yellow,Blue); pozy+=24;
		Dispstr(fk8save*16,1,pozy,pozx,Yellow,Blue);
		for(i=1;i<4;i++) {
			pozx=CDpozX[i]; pozy=CDpozY[fk8save];
			Dispstr(fk8save*16,1,pozy,pozx,Blue,Blue); pozy+=24;
			Dispstr(fk8save*16,1,pozy,pozx,Blue,Blue);
		}
	}	
	dstr=CDMenu; 
	for(i=0;i<4;i++){
		pozx=CDpozX[i]; pozy=CDpozY[mfk8];
		if(i==0) {fcol=Yellow; bcol=Green;} else {fcol=White;bcol=Blue;}
		if(i==mfk4) {fcol=Red; bcol=Green;}
		if((fk4save==3)&&(i!=0)) {fcol=Blue; fcol=Blue;}
		Dispstr(mfk8*16,1,pozy,pozx,bcol,fcol); pozy+=24;
		Dispstr(mfk8*16,1,pozy,pozx,bcol,fcol);
	}
}	

void Dispmsg(u8 nfk8,u8 nfk4)				
{ //u8 i;
	switch(nfk8){
		case 0x00:dstr=CDMenuMsg0;  break;
		case 0x01:dstr=CDMenuMsg1;  break;
		case 0x02:dstr=CDMenuMsg2;  break;
		case 0x03:dstr=CDMenuMsg3;  break;
		case 0x04:dstr=CDMenuMsg4;  break;
//	case 0x05:dstr=CDMenuMsg5;  break;
	}
	if(nfk4==0)													//ÏÔÊ¾ÌáÊ¾
		Dispstr16(fk4*12,6,CDMsgy,CDMsgx,Blue,Blue);
	else
		Dispstr16(fk4*12,6,CDMsgy,CDMsgx,Blue,Yellow);
}
void Dispinput(void)
{	u8 i;
	if(cEdit<=6) {											//ÏÔÊ¾ÊäÈë
		for(i=6;i>0;i--) {
			Dispzf24(' ',InMsgy,InMsgx+i*16,Blue,Yellow);
		}
		for(i=1;i<=cEditbak;i++) {
			if(i==cEdit) Dispzf24(sEdit[i-1],InMsgy,(InMsgx+(cEditbak-i+1)*16),Blue,Red);
			else Dispzf24(sEdit[i-1],InMsgy,(InMsgx+(cEditbak-i+1)*16),Blue,Yellow);
		}
	}
	else{
		for(i=6;i>0;i--) {
			Dispzf24(' ',InMsgy,InMsgx+i*16,Blue,Yellow);
		}
	}
}


void StartProg(void)
{
  switch (fk8)
  { case M0_SHE_ZHI:  switch (fk4) {
      case MM0_MI_MA:      SheZhi_MiMa();			break;
      case MM0_SHI_ZHONG:  SheZhi_ShiZong();  break;
	    case MM0_DENG_JI:    SheZhi_DengJi();   break; } break;
	  case M1_CHAO_ZUO:   switch (fk4) {
      case MM1_QI_DONG:    ChaoZuo_Qidong();  break;
      case MM1_PING_BI:    ChaoZuo_PingBi(); break;
      case MM1_CHAO_BEI:   ChaoZuo_Bei();     break; } break;
	  case M2_DANG_AN:    switch (fk4) {
      case MM2_BAO_JING:   DangAn_BaoJing();  break;
	    case MM2_PING_BI:    DangAn_PingBi();   break;
	    case MM2_DENG_JI:    DangAn_DengJi();   break; } break;
	  case M3_BIAN_CHENG: switch (fk4) {
	    case MM3_XIAN_SHI:   BianCheng_XianShi();  break;
 	    case MM3_LIAN_DONG:  BianCheng_LianDong(); break;
	    case MM3_JIAN_PAN:   BianCheng_JianPan();  break; } break;
	  case M4_CHE_SHI:    switch (fk4) {
	    case MM4_MO_NI:      CheShi_MiNi();        break;
	    case MM4_WAI_SHE:    CheShi_DaoRu();       break;
	    case MM4_CHE_BEI:    CheShi_DaoChu();      break; } break;
//  case M5_ZHI_JIE:    switch (fk4) {
//    case MM5_PING_BI:    ChaoZuo_PingBi();     break;
//    case MM5_QI_DONG:    ChaoZuo_Qidong();     break;
//    case MM5_FU_WEI:     ChaoZuo_PingBi();      break; } break;
  }
 return;
}

void SheZhi_MiMa(void)
{ u16 i; u8 k,buf[2];
//	SPI_Flash_Read(buf,0x90000,6);
//	i=(buf[0]>>8)+buf[1];i=(buf[2]>>8)+buf[3];i=(buf[4]>>8)+buf[5];
	i=(sEdit[3]-0x30)*1000+(sEdit[2]-0x30)*100+(sEdit[1]-0x30)*10+sEdit[0]-0x30;
	if(i==1111) cPass=1;
	else if(i==9999){
		//ÐÂÔö
		cPass=2;
		Disphzs(0xb5c7,TiShi_y,TiShi_x,Blue,White);						//µÇ
		Disphzs(0xc2bc,TiShi_y,TiShi_x+16,Blue,White);				//Â¼
		Disphzs(0xb3c9,TiShi_y,TiShi_x+16+16,Blue,White);			//³É
		Disphzs(0xb9a6,TiShi_y,TiShi_x+16+16+16,Blue,White);	//¹¦
	}
	else if(i==3333) cPass=3;
  else if(i==1234)
	{	cPass=4;
		dstr=DLMenu;Dispstr(9,2,407,564,White,Black);//»Ö¸´³ö³§¼ÇÂ¼ÉèÖÃ
//	SPI_Flash_Read(buf,0x90000,6);
//		buf[0]=0xff;buf[1]=0xff;buf[2]=0xff;buf[3]=0xff;buf[4]=0xff;buf[5]=0xff;
		for(k=0;k<13;k++){ SPI_Flash_Erase_Sector(128+k); } //erase HJ_LD_GZ_ZZ 13page
	}
	else if((i>=1601)&&(i<=1616))
	{	Lp_max=(sEdit[1]-0x30)*10+sEdit[0]-0x30;USART1_SendByte(Lp_max);
		buf[0]=Lp_max;
		SPI_Flash_Write(buf,0x91014,1);  //0x91000+20
//USART1_SendByte(buf[0]);
	}
	else if(i==5380){ cPass=9;	Set_point();} 
	else cPass=0;
//	if(cPass==3) { KZSG_on();} 	else { KZSG_off();} //ZQP_RunXu
//	USART1_SendByte(cPass);

}
//for(k=0;k<6;k++)time[k]=shu[2*k]*10+shu[2*k+1];date=shu[4]*10+shu[5];
//       RTC_DateStructure.RTC_Year=ctime[4]=time[0];	//Äê
//       RTC_DateStructure.RTC_Month=ctime[3]=time[1];//ÔÂ
//       RTC_DateStructure.RTC_Date=ctime[2]=time[2];	//ÈÕ
//       RTC_TimeStructure.RTC_Hours=ctime[1]=time[3]; 	//Ê±
//       RTC_TimeStructure.RTC_Minutes=ctime[0]=time[4];//·Ö
//       RTC_DateStructure.RTC_WeekDay=ctime[5]=time[5];	
//			 date=ctime[2];////////////////µ÷ÊÔ³ÌÐò
//			 BKP_Write();				          
//       status1=RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
//			 status2=RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

void SheZhi_ShiZong(void)
{ if(cPass>1)  //sEdit[5]_first_input_1
	{	u8 year,mon,day,hour,min,status1,status2;
		RTC_TimeTypeDef RTC_TimeStructure;
		RTC_DateTypeDef RTC_DateStructure; 

		Ds1302Write(0x8E,0x00);		 //Dis Write BaoHu

		if((sEdit[5]==0x31)&&(sEdit[4]==0x30)) //set time
		{ hour= (sEdit[3]-'0')*16 + (sEdit[2]-'0');
			min= (sEdit[1]-'0')*16 + (sEdit[0]-'0');
			Ds1302Write(0x84,hour);		 // W_hour
			RTC_TimeStructure.RTC_Hours=hour;
			ctime[1]=hour; 	//Ê±
			Ds1302Write(0x82,min);		 // W_minu
			RTC_TimeStructure.RTC_Minutes=min;
			ctime[0]=min;	//·Ö
			BKP_Write();				          
//       status1=RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
//USART1_SendByte(hour);Delay16s(20,20);USART1_SendByte(min);
		}
		else  //set date
		{	if(sEdit[5]==0x32)
			{	year=(sEdit[5]-'0')*10 + (sEdit[4]-'0');
				mon= (sEdit[3]-'0')*16 + (sEdit[2]-'0');
				day= (sEdit[1]-'0')*16 + (sEdit[0]-'0');
				Ds1302Write(0x8c,year); RTC_DateStructure.RTC_Year=year;	ctime[4]=year;	//Äê
				Ds1302Write(0x88,mon); RTC_DateStructure.RTC_Month=mon;	ctime[3]=mon;	//ÔÂ
				Ds1302Write(0x86,day); RTC_DateStructure.RTC_Date=day;	ctime[2]=day;	//ÈÕ
				BKP_Write();				          
				status2=RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
//USART1_SendByte(year);Delay16s(20,20);USART1_SendByte(mon);Delay16s(20,20);
//USART1_SendByte(day);
			}
			if(sEdit[5]==0x34) //RTC_ini
			{
				RTC_Configuration();// RTC ÅäÖÃ		 
				RTC_Time_Init();

				BKP_Write();				          

//				USART1_SendByte(0xde);
			}
		}
//ResetDate.da_year 
//ResetDate.da_mon 
//ResetDate.da_day 

//		dstr=TMenu;Dispstr(0,8,407,380,White,Black); //´òÓ¡ cPass low
		status2=status2+status1;
//USART1_SendByte(year); USART1_SendByte(mon); USART1_SendByte(day);
//USART1_SendByte(hour); USART1_SendByte(min);
	}
	else
	{	USART1_SendByte(0xb1); }
}
void SheZhi_DengJi(void)
{ u8 con,ll,aa;
	if(cPass>=2)
	{	//USART1_SendByte(0xb2);
		con=(sEdit[5]-'0');
		ll=(sEdit[4]-'0')*10 + (sEdit[3]-'0');
		aa=(sEdit[2]-0x30)*100+(sEdit[1]-0x30)*10+sEdit[0]-0x30;
		DengjiZD(con,ll,aa);
	}
	else
	{ //print deng_lu
	}	
}
void ChaoZuo_Qidong(void)
{	u8 ll,aa,djbc;
	ll=(sEdit[4]-'0')*10 + (sEdit[3]-'0');
	aa=(sEdit[2]-'0')*100 + (sEdit[1]-'0')*10 + (sEdit[0]-'0');	
	djbc=djb[ll-1][aa];
	if(sEdit[5]==0x31)
	{	if(ll<=16)
		{	if((djbc&0x20)==0x00)
			{	djbc |=0x20; lpxq_bz[ll-1]=0x02; //VZQD++;
				HJalarm(3,0,ll,aa);
				U7_QDHDcmd(3,(ll-1)*256+aa);
//				CRT_alarm(3,ll,aa,0);                //QD_CRT
//				USART1_SendByte(0xb3);
			}
		}	
		if(ll==17)
		{	if((djbc&0x20)==0x00)
			{	//djbc |=0x20; 
//				lpxq_bz[ll-1]=0x02; //VZQD++;
//				HJalarm(3,0,ll,aa);
				if((aa&0x01)==1)	{ U7_txQD[2][(aa-1)/2] |=0x40; }
				else      				{ U7_txQD[2][(aa-1)/2] |=0x04; }
				
//				CRT_alarm(3,ll,aa,0);                //QD_CRT
//USART1_SendByte(U7_txQD[2][0]);Delay16s(20,20);
//USART1_SendByte(0xb3);
			}
		}	
	}	
	if(sEdit[5]==0x32)
	{	if(ll<=16)
		{	if((djbc&0x20)==0x20)
			{	djbc &=0xdf; lpxq_bz[ll-1]=0x02; 
//				if(VZQD>0) VZQD--;
				HJalarm(4,0,ll,aa);
				U7_QDHDcmd(4,(ll-1)*256+aa);
//				CRT_alarm(4,ll,aa,0);                //QD_CRT
//USART1_SendByte(0xfb);
			}
		}	
		if(ll==17)
		{	if((djbc&0x20)==0x20)
			{	//djbc &=0xdf; 
				//lpxq_bz[ll-1]=0x02; 
//				if(VZQD>0) VZQD--;
//				HJalarm(4,0,ll,aa);
//				CRT_alarm(4,ll,aa,0);                //QD_CRT
//				ut2_subnum=1;ut2_cmd=0x65;ut2_key=aa+(ll-1)*0x20;ut2_keyb=0xff;
//				USART1_SendByte(0xb4);
			}
		}	
	}
	if((sEdit[5]>=0x33)&&(sEdit[5]<=0x39))		//3->Num_1_quyu 
	{	if(ut6_jz==0)  													//is Ji Zong Ji
		{	
//USART1_SendByte(0x43);			
//			if((djbc&0x20)==0x20)
			{	ut6_num =sEdit[5] -0x32;	//33->1_quyu 
//				ut6_ll =ll; ut6_aa =aa;
//				ut6_cmd =0x63; //bz
				QuYu_alarm(0x63,ll,aa,ut6_num);
				HJalarm(3,ut6_num,ll,aa);
//				U7_QDHDcmd(4,(ll-1)*256+aa);
//				CRT_alarm(4,ll,aa,0);                //QD_CRT
//USART1_SendByte(ut6_num);
			}
		}	
	}
	djb[ll-1][aa]=djbc;
}
void ChaoZuo_PingBi(void)
{	u8 dll,daa,dat[4];
	u32 daddr;
//USART1_SendByte(0xb4);
	dll=(sEdit[4]-'0')*10 + (sEdit[3]-'0');
	daa=(sEdit[2]-'0')*100 + (sEdit[1]-'0')*10 + (sEdit[0]-'0');	
	daddr=0x8d000+(dll-1)*256+daa;
	if(sEdit[5]==0x31)
	{	SPI_Flash_Read(dat,daddr,1);
		if((dat[0]&0xc0)==0x80)
		{	dat[0] |=0x40; djb[dll-1][daa]|=0x40; VZPB++;
			SPI_Flash_Write(dat,daddr,1);   
			HJalarm(32,0,dll,daa);
//			CRT_alarm(32,dll,daa.0);
//USART1_SendByte(dat[0]);
		}
	}	
	if(sEdit[5]==0x32)
	{	SPI_Flash_Read(dat,daddr,1);
		if((dat[0]&0xc0)==0xc0)
		{	dat[0] &=0xbf; djb[dll-1][daa]&=0xbf;
			if(VZPB>0) VZPB--;
			SPI_Flash_Write(dat,daddr,1);
//			SPI_Flash_Read(dat,0x8f00+4095,1); Read4095 -1
			HJalarm(33,0,dll,daa);
//			CRT_alarm(33,dll,daa,0);
//USART1_SendByte(dat[0]+1);
		}
	}	
//if((djb[dll][daa]&0xc0)==0x80){Dispstr(5,1,407,688,White,Black);djb[dll][daa]|=0x40;HJalarm(32,0,dll+1,daa);goto loop1;}
//if((djb[dll][daa]&0xc0)==0xc0){Dispstr(8,1,407,688,White,Black);djb[dll][daa]&=0xbf;HJalarm(33,0,dll+1,daa);}
}
void ChaoZuo_Bei(void)
{
//	USART1_SendByte(0xb5);
}
void DangAn_BaoJing(void)
{
	chaType=1; ChaHJpg=0;
	hjjlfg=ldjlfg=qtjlfg=0;
	jlend=0;
	Displine_JLB(dispZON,0);
//USART1_SendByte(0xb6);
}
void DangAn_PingBi(void)
{	chaType=2;
  PBBcx(0);
//	USART1_SendByte(0xb7);
}
void DangAn_DengJi(void)
{	chaType=3;
	DJBcx(1);
//	USART1_SendByte(0xb8);
}
void BianCheng_XianShi(void)
{
//	USART1_SendByte(0xb9);
}
void BianCheng_LianDong(void)
{
//	USART1_SendByte(0xba);
}
void BianCheng_JianPan(void)
{
//	USART1_SendByte(0xbb);
}
void CheShi_MiNi(void)
{	
	//ÐÂÔö
	chaType=5;
	moniLL=(sEdit[1]-0x30)*10+sEdit[0]-0x30;
	if(moniLL!=0) moniBZ=1; else moniBZ=0;
}
void CheShi_DaoRu(void)
{	u16 i; 
	u8 j;
//	USART1_SendByte(0xbd);
	i=(sEdit[5]-'0')*1000 + (sEdit[4]-'0')*100 + (sEdit[3]-'0')*10 + (sEdit[2]-'0');
	j=(sEdit[1]-0x30)*10+sEdit[0]-0x30;
	if(i==1234) { DownL=2; Download_PC();}	//Uart0ÏÂÔØ
	if(i==2345) { USB_HOST_MAIN(j);}				//USBÏÂÔØ

}
void CheShi_DaoChu(void)
{ u16 i; u8 j;
//	USART1_SendByte(0xbe);
	i=(sEdit[5]-'0')*1000 + (sEdit[4]-'0')*100 + (sEdit[3]-'0')*10 + (sEdit[2]-'0');
	j=(sEdit[1]-0x30)*10+sEdit[0]-0x30;
	if(i==4321) Upload_PC(j);	//shang_chuan
}

void KZSG_199(void)
{ //u8 ll=1,aa=199;
	if(Spalarm==1)
	{ 
//		HJalarm(3,0,ll,aa); djb[ll-1][aa]|=0x20; lpxq_bz[ll-1]=0x02;
//		U7_QDHDcmd(3,(ll-1)*256+aa);
	}
	else 
	{ 
//		HJalarm(4,0,ll,aa); djb[ll-1][aa]&=0xdf; lpxq_bz[ll-1]=0x02;
//		U7_QDHDcmd(4,(ll-1)*256+aa);
	}
}
//void ChaoZuo_PingBi(void)
//{	u8 i;
//	fk8=5; fk4=1; bProg=1;
//	cEdit=DigitNum[fk8][fk4];				//²ÎÊý¸öÊý
//	cEditbak=cEdit;	
//	Dispmsg(fk8,fk4);								//ÏÔÊ¾ÌáÊ¾		
////	DispMenu(fk8,fk4);							//ÏÔÊ¾²Ëµ¥
//	Dispinput();			 							//ÏÔÊ¾ÊäÈëÊý
//	for(i=0;i<7;i++) sEdit[i]=35;  // ******
//	fk8save=fk8;
//	USART1_SendByte(0x33);
//	i=(sEdit[5]-'0')*10 + (sEdit[4]-'0');
//	USART1_SendByte(i);
//}

 
//ÅäÖÃËùÓÐÍâÉè
void Init_All_Periph(void)
{  
 	RCC_Configuration();		
	GPIO_Configuration();
	Ur485_1=0;Ur485_2=0;SPI1_CS =1;
	GPIOI->BSRRH = 0x0000;//GPIOH->BSRRH=0xff00; 
	USART1_Configuration();
	USART2_Configuration();
	USART3_Configuration();
	UART4_Configuration();
	USART6_Configuration();
	UART7_Configuration();
  TIM6_Configuration();
//	RTC_Configuration();    !!! LINSHI
	SDRAM_Configuration();
	SPI1_Configuration();
	//CAN1_Configuration();
	NVIC_Configuration();
	/*³õÊ¼»¯Òº¾§ÆÁ*/
  Lcd_ini();
  LCD_LayerInit();
  LTDC_Cmd(ENABLE);
	ZQbuffer|=0x8;//±³¹âÁÁ
	Prnt_ini(); 
	//IWDG_Configuration(IWDG_Prescaler_256 ,0xfff); 
}

void Delay16s(u16 delx,u16 circle) //20,20_is13us; 16,1_is0.61us
{ u16 i=0;
  for(;circle>0;circle--)
    {i=0;while(i<delx){i++;}}
}
void Delay1ms(u16 dms)
{ u16 i;
	for(i=0;i<dms;i++)
	{	Delay16s(70,500);}
}
void Read_sys(void)
{	u8 i,k,buf[32],sumz[16];
	u16 j,sums=0;
	u32 waddr;
 //¶ÁÈ¡FLASH_ID
 ID=SPI_Flash_ReadID();
 if(ID==0x14EF) Dispzf24('S',200,500,Magenta,Yellow);else Dispzf24('E',200,500,Magenta,Yellow);
 counter=3200;llpp=8;// Read_point();
//¶Á²¦Âë¿ª¹Ø×´Ì¬
	Boma_on();Delay16s(20,20);
	j=GPIOI->IDR;
	Ledscan_off();
  Boma=j>>8;
	i=(Boma&0x80)>>7; ut6_jz=i; // ¼¯ÖÐ»ú=0 ÇøÓò»ú=1
  i=(Boma&0x60)>>5; ut6_max=i+1; //when jz=0 ÇøÓòÊýÁ¿ when jz=1_ÇøÓò»úºÅ
//USART1_SendByte(ut6_max);Delay16s(20,20);USART1_SendByte(ut6_jz);
	if((Boma&0x02)==0) {hjcycle=1; gzcycle=25; }
	else 							{ hjcycle=5; gzcycle=85; }
	fire_lo=117; fire_hi=118;	//if((Boma&0x20)==0){ fire_lo=117; fire_hi=118; }
														//else { fire_lo=80; fire_hi=130; }
//USART1_SendByte(fire_hi);Delay16s(20,20);
	DownL=2;//ÏÂÔØ	
//ÀúÊ·¼ÇÂ¼Ö¸Õë
  SPI_Flash_Read(buf,0x8c000,12);
  hjjlzz=buf[0]<<8;hjjlzz+=buf[1];hjjltrue=buf[2]<<8;hjjltrue+=buf[3];
  if(hjjlzz==0xffff) hjjlzz=0;
  if(hjjltrue==0xffff) hjjltrue=0;
	ldjlzz=buf[4]<<8;ldjlzz+=buf[5];ldjltrue=buf[6]<<8;ldjltrue+=buf[7];
  if(ldjlzz==0xffff) ldjlzz=0;
  if(ldjltrue==0xffff) ldjltrue=0;	
  tjlzz=buf[8]<<8;tjlzz+=buf[9];tjltrue=buf[10]<<8; tjltrue+=buf[11];
  if(tjlzz==0xffff) tjlzz=0;
	if(tjltrue==0xffff) tjltrue=0;
//ÃÜÂë
  SPI_Flash_Read(buf,0x90000,6);
  pass[0]=buf[0]<<8;pass[0]+=buf[1];
  if(pass[0]==0xffff) pass[0]=1111;
  pass[1]=buf[2]<<8;pass[1]+=buf[3];
  if(pass[1]==0xffff) pass[1]=2222;
  pass[2]=buf[4]<<8;pass[2]+=buf[5];
  if(pass[2]==0xffff) pass[2]=3333;	
//ÏµÍ³Éè¶¨
  SPI_Flash_Read(buf,0x91000,32);
  printer=1;//if(buf[0]<2)printer=buf[0];else printer=0; //print on off
	
  if(buf[1]<2)bdjc=buf[1];else bdjc=0;  				//BeiDian_JianCha     
  if(buf[2]<2)sgjc=buf[2];else sgjc=0;  				//ShengGuang_JianCha    
  if(buf[4]<32)machine=buf[4];else machine=0; 	//BenJi_DiZhi
  if(buf[16]<2)CRTxj=buf[16];else CRTxj=0;    	//CRT XunJian   
  if(buf[17]<8)zxztb[0]=buf[17];else zxztb[0]=0; //ZhuanXianPan
//×ÜÏßÅÌ¼ü±í
  for(k=0;k<3;k++) 
	{ for(i=0;i<128;i++)
		{ SPI_Flash_Read(buf,0x9c000+i*2,2);
			j=buf[0]*256+buf[1];
			KBnumadr[k][i]=j;
		}
	}
//Éù¹â¹ã²¥±í
 for(i=0;i<32;i++)
	{ SPI_Flash_Read(buf,0x9d000+i*32,32);
		for(k=0;k<16;k++)
		{	j=buf[k*2]*256+buf[k*2+1];
			SGGB[i*16+k]=j;
		}
	}
// for(j=0;j<510;j++)
// {	USART1_SendByte(SGGB[j]/256);Delay16s(20,20);
//	  USART1_SendByte(SGGB[j]&0x00ff);Delay16s(20,20);
// }
//  SPI_Flash_Read(buf,0x91014,1);	
//  if( buf[0]<=16 ) Lp_max=buf[0];	else Lp_max=16; //Denglu_16xx LpNum
//USART1_SendByte(Lp_max);		
//µÇ¼Ç±í
  SPI_Flash_Read(downb,0x8d000,4096);
  for(i=0;i<16;i++) sumz[i]=0;
  for(k=0;k<16;k++)
  { j=0;
    if(downb[k*256+201]==0xff)
    { for(i=0;i<=200;i++)djb[k][i]=0; djb[k][201]=0;}	   //!! yuan djb[k][201]=0;
    else  
		{ djb[k][201]=downb[k*256+201];
			for(i=0;i<=200;i++)
			{ if(downb[k*256+i]<=0xc1 && downb[k*256+i]>=0x80)
				{ djb[k][i]=downb[k*256+i];sumz[k]++;sums++;
					if(djb[k][i]==0x90)lpcxsum++;
					if(counter==sums)goto loop;
				}
			}
	  }
	}
	VZZQDJ =0;
	for(i=0;i<32;i++)	//ZQP_DJ at:Loop16_+216 to 248 read_sys is 1-32
	{	k =downb[15*256+216+i];
		if(k==0xff) { k=0; }
		if(k!=0) 		{	VZZQDJ++;}
		djb[16][i+1] =k;			//aa+1 addr from 0-31 to 1-32
//USART1_SendByte(k);		
	}				
loop:
	for(i=0;i<16;i++)djb[i][201]=sumz[i];
	VZDJ=sums;
  for(i=0;i<100;i++){for(j=0;j<6;j++) strblp[i][j]=0;} 
  for(i=0;i<8;i++)    
  { 
//USART1_SendByte(djb[i][201]);		
//		if(djb[i][201]>0)
		{ strblp[i][0]=0xF0;strblp[i][1]=0x01;strblp[i][2]=0xF2;strblp[i][5]=i;
		  strblp[i*10+8][0]=0xF0;strblp[i*10+8][1]=0x04;strblp[i*10+8][2]=0xF2;
			strblp[i*10+8][5]=i;              
      strblp[i*10+9][0]=0xF0;strblp[i*10+9][1]=0x03;strblp[i*10+9][2]=0;
			strblp[i*10+9][3]=0xF2;strblp[i*10+9][5]=i;
	  }
//    else
//    { strblp[i][0]=0;strblp[i][1]=0;strblp[i][2]=0;strblp[i][5]=0;}		
	}
	for(i=1;i<8;i++)
  { for(j=0;j<8;j++)
	  { strblp[10*i+j][0]=strblp[j][0];strblp[10*i+j][1]=strblp[j][1];
		  strblp[10*i+j][2]=strblp[j][2];strblp[10*i+j][5]=strblp[j][5];
		}
	}	
		
//ÆÁ±Î±í
  SPI_Flash_Read(downb,0x8f000,4096);
  if(downb[4095]<=64) xspbs=downb[4095];else xspbs=0; 
  if(xspbs>0)
  {for(i=0;i<xspbs;i++){for(k=0;k<8;k++)Disp_pb[i][k]=downb[i*8+k];}}

	for(k=0;k<=16;k++)		//read name to namb[][]  =125XHS
	{	for(i=0;i<=200;i++)
		{	waddr=0xa0000+(k)*6400+(i)*32;
			SPI_Flash_Read(buf,waddr,3);
			if(buf[0]>0x80) buf[0]=buf[0]-0x40;		//LQL
			namb[k][i+1] =buf[0];				//namb[][1]=addr1
//if((k==0)&&(i==0)) USART1_SendByte(namb[k][i+1]);					
		}
	}	
}


void Rfsh(void)
{
 u8 i,j,u,d;
 u8 sll,saa; //,j,y  temp,z
 u16 k,m;
 Tled++;
//iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
// if(VZQDHD==1)
///{ if((Tled%2)==0)	{LedLD_on();}
//	 else							{LedLD_off();}	
// }	
 DispFactor();

 //Ë¢ÐÂ×´Ì¬
 i=1;Dispsz16(VZHJ,4,sys_1y,sys_1x+i*40,Magenta,White);   //»ð¾¯
 i=2;Dispsz16(VZQD,4,sys_1y,sys_1x+i*40,Magenta,White);   //Æô¶¯
 i=3;Dispsz16(VZHD,4,sys_1y,sys_1x+i*40,Magenta,White);   //·´À¡
 i=4;Dispsz16(VZDJ,4,sys_1y,sys_1x+i*40,Magenta,White);   //µÇ¼Ç
 i=5;Dispsz16(VZGZ,4,sys_1y,sys_1x+i*40,Magenta,White);   //¹ÊÕÏ
 i=6;Dispsz16(VZPB,4,sys_1y,sys_1x+i*40,Magenta,White);   //ÆÁ±Î
 i=7;Dispsz16(VZSX,4,sys_1y,sys_1x+i*40,Magenta,White);   //ÉÏÏß
 i=8;Dispsz16(cPass+VZZQDJ*100,4,sys_1y,sys_1x+i*40,Magenta,White);   //cPass

	dstr=POWmenu;    //POWmenu[]=("Õý³£Ö÷µç±¸µçÊÖ¶¯×Ô¶¯µçÔ´¹ÊÕÏ");
	if(K_BDGZ==0)    //      u,d   0   4   8   12  16  20  24
	{	if(K_ZDGZ==0x10) {u=0;d=20;}   //Õý³£
		else             {u=24;d=4;}	 //Ö÷¹Ê
	}
	else               {u=24;d=8;}	 //±¸¹Ê
	i=9;          Dispstr16(u,2,sys_1y,sys_1x+i*40,Magenta,White);
	dstr=POWmenu; Dispstr16(d,2,sys_2y,sys_1x+i*40,Magenta,White);
	dstr=POWmenu;
	if(K_RXS==0) j=16; else j=12;
	i=10;Dispstr16(j,2,sys_1y,sys_1x+i*40,Magenta,White);
// i=9;Dispsz16(VZLP,4,sys_1y,sys_1x+i*40,Magenta,White);   //
	for(i=0;i<8;i++)Dispsz16(LP_Num[i],3,sys_1y,sys_2x+i*27,Magenta,White);	//1-8sub
	for(i=0;i<8;i++)Dispsz16(LP_Num[i+8],3,sys_2y,sys_2x+i*27,Magenta,White);	//9-16sub
			
//YX and Led 

	if(VZHJ==0)
	{ if((VZQD+VZHD+VZZQ)!=0)	{KYXLD_on();	} 
		else
		{	if(VZGZ!=0) {	KYXGZ_on();}
			else 				{	KZYX_off();}
		}
	}
	else { KYXHJ_on();}
	if(Kyxbz==1) {KZYX_off();}
	if(VZGZ==0)	{LedGZ_off();} else {LedGZ_on();}
	if(VZQD==0)	{LedLD_off();} else {LedLD_on();}
	if(VZHD==0)	{LedFQ_off();} else {LedFQ_on();}
	
//	if(cPass==3) { KZSG_on();}//	else { KZSG_off();} 		//ZQP_RunXu
	
	//Ö÷±¸µç¼ì²â
//	if((Boma&0x02)==0x02)
		if((K_ZDGZ&0x10)==0x10)  //ZDZC
		{ if(zdgz==0x80)
			{	zdgz=0; LedZDGZ_off();
//				if(bdgz!=0x80)LedBDZC_off();
				HJalarm(21,0,17,193); VZGZ--;
//USART1_SendByte(K_ZDGZ+1);Delay16s(20,20);				
			}
		}
		else											//ZDGZ
		{ if(zdgz!=0x80)
			{ zdgz++;  
				if(zdgz==10)  //gzcycle=85/125
				{ LedZDGZ_on(); //LedZDZC_off(); 
					zdgz=0x80;HJalarm(16,0,17,193);VZGZ++;
//					if(bdgz!=0x80) LedBDZC_on(); 
//USART1_SendByte(K_ZDGZ);Delay16s(20,20);	
				}	 
			}
		}
		if((K_BDGZ&0x20)==0x00)       //BDZC
		{ if(bdgz==0x80)
			{ bdgz=0; LedBDGZ_off() ;
//					if(zdgz==0x80)LedBDZC_on();
				HJalarm(21,0,17,194); VZGZ--;
//USART1_SendByte(K_BDGZ+1);Delay16s(20,20);	
			}
		}
		else
		{	if(bdgz!=0x80)
			{	bdgz++;  
				if(bdgz==10) //gzcycle=85/125
				{	LedBDGZ_on() ;//LedBDZC_off();
					bdgz=0x80;HJalarm(16,0,17,194); VZGZ++;
				}
//USART1_SendByte(K_BDGZ);Delay16s(20,20);	
			}
		}
	
//ÊÖ¶¯/×Ô¶¯Ëø¼ì²â
	if((K_RXS &0x80)==0x80) //ÊÖ¶¯ open
	{	autofg=0;LedZD_off();LedSD_on();
	}
	else								//×Ô¶¯Ëø short
	{	autofg=1;LedSD_off();LedZD_on();
	} 			 

	if(Tled>=60)   //60    ¹Ø±³¹â     
	{	Tled=0;
		if((hjflag+ldflag+gzflag)==0)Tlcd++;
		if(Tlcd>=20)
		{	Tlcd=0;KLCD_off();
			BKP_Write();				          
		}//5min¹Ø±³¹â 
    
		Get_time();
		DispTimeUP();
//USART1_SendByte(K_RXS);Delay16s(20,20);
	}
  if(autofg>0&&ldqdfg>0)			//Âú×ãÆô¶¯Ìõ¼þ,Á¢¼´Æô¶¯
	{	for(k=0;k<200;k++)
		{	if(cxb[k]==0x80)
			{liandongQD(k,1);cxb[k]=0xff;ldqdfg--;
			}
		}
	}
//Éù¹â¹ã²¥¼ì²â
	Tsggb++;
 if(((Boma&0x01)==0x00)&&(bzSGGB ==1))		//Boma_1:Éù¹â¹ã²¥ on/off
 {	
	if(Tsggb==18)   //15 Sec SG_off GB_on 
	{	if(VZQD !=0)
		{	for(m=1;m<512;m++)
			{ sll=SGGB[m]/256;saa=SGGB[m]&0x00ff;
//USART1_SendByte(sll);Delay16s(20,20);USART1_SendByte(saa);				
				if(sll >0x80)
				{	sll &=0x7f;
					djb[sll-1][saa] &=0xdf;
//USART1_SendByte(sll);Delay16s(20,20);USART1_SendByte(saa);Delay16s(20,20);
				}
			}
//			USART1_SendByte(0xc0);
			KZHJ_on();			//»ð¾¯¼ÌµçÆ÷ on			
		}
	}
	if(Tsggb==46)   //33 Sec  SG_on GB_off    
	{	if(VZQD !=0)
		{	for(m=1;m<512;m++)
			{ sll=SGGB[m]/256;saa=SGGB[m]&0x00ff;
//USART1_SendByte(sll);Delay16s(20,20);USART1_SendByte(saa);				
				if(sll >0x80)
				{	sll &=0x7f;
					djb[sll-1][saa] |=0x20;
//USART1_SendByte(sll);Delay16s(20,20);USART1_SendByte(saa);Delay16s(20,20);
//USART1_SendByte(djb[sll-1][saa]);	Delay16s(20,20);	
				}
			}
			KZHJ_off();			//»ð¾¯¼ÌµçÆ÷ off			
		}
	}
	if(Tsggb>=50)   //0 Sec    
	{	Tsggb =0;
	}
//USART1_SendByte(Tsggb);Delay16s(20,20);
 }
 else
 { //if(VZQD !=0) { KZHJ_on();	}		//»ð¾¯¼ÌµçÆ÷ on
	 //else         { KZHJ_off();  }   //»ð¾¯¼ÌµçÆ÷ off
 }
	
}

void Pross_hj(void)
{ u8 ll,aa,djbc,val,bits;
	
//	u32 waddr;
//	lpflag_hj=0;qdhdfg=0;
	VZSX=0; VZQDHD=0; 
	for(ll=0;ll<16;ll++)    //val[0]=20 
	{	LP_Num[ll]=0;
		
		if((lpxz_zt[ll]&0x80)==0)
		{	if((lpxz_val[ll][0]&0xfe)==0x30) 	lpgzjs[ll]++; else lpgzjs[ll]=0;
			if(lpgzjs[ll]>=4)	//gzcycle=60
			{	lpxz_zt[ll]|=0x80; lpgzjs[ll]=0;
				HJalarm(16,0,135,ll+1); VZGZ++;
//				CRT_alarm(16,0,ll+1,0);
//USART1_SendByte(0x66);Delay16s(16,2);				
			}
		}	
		if((lpxz_zt[ll]&0x80)==0x80)
		{	if((lpxz_val[ll][0]&0xfe)!=0x30)
			{	lpxz_zt[ll]&=0x7f; lpgzjs[ll]=0;
				HJalarm(21,0,135,ll+1); VZGZ--;
//				CRT_alarm(21,0,ll+1,0);
			}
		}	
		if((lpxz_zt[ll]&0x80)==0)	//lpxz_val[ll][0]==0x20&&»ØÂ·Õý³£
		{	for(aa=1;aa<=200;aa++)
			{	djbc=djb[ll][aa];val=lpxz_val[ll][aa];bits=lpxz_bit[ll][aa];
				if((djbc&0xc0)==0x80)	//dj_nopb
				{	//if(autofg>0&&unasw[ll][aa]==0x80){unasw[ll][aa]=0;liandongBC(ll+1,aa);}//ËøÔÊÐíºóÁª¶¯
					if((djbc&0x03)==0x00)					//Ì½²âÆ÷ »ð¾¯
					{	
						if((val>=fire_lo)&&(val<fire_hi)&&(bits!=0))  //val>=0x38
						{	hjjs[ll][aa]++;gzjs[ll][aa]=0;
							if(hjjs[ll][aa]>=hjcycle)
							{	djbc |=0x02;hjjs[ll][aa]=0; VZHJ++;
								lpxq_bz[ll]=0x02;
								haveHJ=1;	LedHJ_on();
								djbc |=0x20;  //KYXLD_on();KZYX_on(); //linshi_diandeng
								if(!hjflag)HJalarm(0,0,ll+1,aa);else HJalarm(1,0,ll+1,aa);//»ð¾¯
//								if(autofg>0)liandongBC(ll+1,aa);else unasw[ll][aa]=0xff;
								liandongBC(ll+1,aa);
								if(ut6_jz==0)	{CRT_alarm(1,ll+1,aa,0); }	//jizhong=0 quyu=1
								else					{ QuYu_alarm(0x42,ll+1,aa,ut6_max); }
								CengX_alarm(1,ll+1,aa);
							}
						}
						else { hjjs[ll][aa]=0; }       //clear HJ count!!!
					}
					if((djbc&0x04)==0x04)     //¹ÊÕÏ»Ö¸´
					{	if(val>=3)
						{	HJalarm(21,0,ll+1,aa); //fault_repair
							djbc&=0xfb; VZGZ--;
							gzjs[ll][aa]=0;
								if(ut6_jz==0)	{CRT_alarm(21,ll+1,aa,0); }
								else					{ QuYu_alarm(0x69,ll+1,aa,ut6_max); }
								if(ll==0)	{	if(aa==199) LedSGGZ_off(); }									
						}
					}						
					if((djbc&0x4)==0)					//¹ÊÕÏ
					{
						{	if(val<3){
								gzjs[ll][aa]++;
						}else gzjs[ll][aa]=0;
//	USART1_SendByte(gzjs[ll][aa]);Delay16s(16,2);
							if(gzjs[ll][aa]>gzcycle)
							{	HJalarm(16,0,ll+1,aa);	//fault
								djbc|=0x04; VZGZ++; 
								gzjs[ll][aa]=0;
								if(ut6_jz==0)	{CRT_alarm(16,ll+1,aa,0); }
								else					{ QuYu_alarm(0x67,ll+1,aa,ut6_max); }
								if(ll==0)
								{	if((aa==199)||(aa==198)) LedSGGZ_on();
								}									
							}
						}
					}						

					if(((djbc&0x03)==0x01)||((djbc&0xa3)==0xa1))		//Ä£¿é
					{	if((val>=fire_lo)&&(val<fire_hi)&&(bits!=0)) 	// 125Ïû»ðË¨val=15
						{	hjjs[ll][aa]++;gzjs[ll][aa]=0;
							if(hjjs[ll][aa]>=10)   //hjcycle=5
							{	djbc |=0x02;hjjs[ll][aa]=0; VZHD++;
								HJalarm(6,0,ll+1,aa);//»Ø´ð
								U7_QDHDcmd(6,(ll)*256+aa);
								lpxq_bz[ll]=0x02;
								haveHJ=1;
								liandongBC(ll+1,aa);
//								CRT_alarm(6,ll+1,aa,0);
								unasw[ll][aa]=0x80;
							}
						}
					}
					if(((djbc&0x03)==0x03)||((djbc&0xa3)==0xa3))		//Ä£¿é
					{	if((((namb[ll][aa])!=125)&&(val<0x50))||(((namb[ll][aa])==125)&&(bits==0)))
						{	djbc &=0xfd;hjjs[ll][aa]=0; VZHD--;
							HJalarm(7,0,ll+1,aa);//»Ø´ð³·Ïú
							U7_QDHDcmd(7,(ll)*256+aa);
//							CRT_alarm(7,ll+1,aa,0);
						}
					}
					if(((djbc&0x01)==0x01)||((djbc&0xa1)==0xa1))	//Ä£¿é
					{	if((djbc&0x20)==0x20)												//QD ed
						{	if(aa!=199)																//No Sengguang
							{	if((djbc&0x02)!=0x02) VZQDHD=1;	}				//Æô¶¯!=»Ø´ð
						}	
					}
				}			 
				djb[ll][aa]=djbc;
				if((val>4)&&(val<0xa0)){
					LP_Num[ll]++;
					
				} 
			}	//for(aa=1;aa<=200;) end	 
		}		//if(xz[ll][0]==0x20 »ØÂ·Õý³£ end
		VZSX=VZSX+LP_Num[ll];
	}			//for(ll=0;ll<16;ll++) end
	for(ll=16;ll<18;ll++)    //ZhiQiPan QD flash//Ä£¿é
	{	for(aa=1;aa<=200;aa++)
		{	djbc=djb[ll][aa];
//			if((djbc&0x04)==0x00)		    //No GZ
			{	if((djbc&0x02)==0x02)												//QD ed
				{	if((djbc&0x01)!=0x01) VZQDHD=1;				//Æô¶¯!=»Ø´ð
				}
			}
		}
	}
	return;
}

void Usart_loop7ZhiQi(u8 dll)			////this  dll=2 ZhiQipan
{	u8 i,KBbuf[64],dat,djbc;

	if(U7_recOK[U7_mode]==2)	//ZQP communicata NO error
	{	
//USART1_SendByte(0xc1);
			for(i=0;i<32;i++)
			{	KBbuf[i*2]  =U7_rxKB[U7_mode][i]>>4;
				KBbuf[i*2+1]=U7_rxKB[U7_mode][i]&0x0f;
			}
//for(j=1;j<33;j++){USART1_SendByte(djb[16][j]);Delay16s(16,2);}
//for(j=0;j<32;j++){USART1_SendByte(KBbuf[j]);Delay16s(16,2);}
			for(i=0;i<32;i++)
			{	
				djbc=djb[16][i+1]; 
				dat=KBbuf[i];
				if((djbc&0x81)==0x81)
				{
					if((dat&0x04)==0x04)			//SouQi 0100
					{
						if((djbc&0x20)==0x00)		//
						{
//USART1_SendByte(i);	Delay16s(16,2);
//USART1_SendByte(0xbb);Delay16s(16,2);
							HJalarm(3,0,17,i+1);	//
							LedLD_on();KYXLD_on();KZYX_on(); //linshi_diandeng
//							CRT_alarm(3,KBll,KBaa,0);                //QD_CRT
							djbc |=0x20; //djb[16][i]=djbc;
						}
					}					
					if((dat&0x04)==0x0)			//SouTing 0100
					{	if((djbc&0x20)==0x20)	//{hjjs[16][i+1]++;}else{hjjs[16][i+1]=0;}if(hjjs[16][i+1]>5)
						{	
								HJalarm(4,0,17,i+1);	//
//for(j=0;j<16;j++){USART1_SendByte(djb[16][j]);Delay16s(16,2);}
//USART1_SendByte(dat);Delay16s(16,2);USART1_SendByte(i);Delay16s(16,2);
								djbc &=0xdf; //djb[16][i]=djbc;
								if((i&0x01)==0) U7_txQD[dll][i/2] &=0xbf;
								else						U7_txQD[dll][i/2] &=0xfb;
//USART1_SendByte(0xa4);
						}
					}				
					if((dat&0x02)==0x02)			//FQ 0010
					{	if((djbc&0x02)==0x00)		//
						{hjjs[17-1][i+1]++;//gzjs[ll][aa]=0;
							if(hjjs[17-1][i+1]>=9)   //hjcycle=5
							{	HJalarm(6,0,17,i+1);	//»Ø´ð
//							CRT_alarm(3,KBll,KBaa,0);                //QD_CRT
								djbc |=0x02; //djb[16][i]=djbc; 
								VZHD++;
							}
//USART1_SendByte(0xa6);
						}
					}					
					if((dat&0x02)==0x00)			//FQ  0010
					{	if((djbc&0x02)==0x02)		//
						{	HJalarm(7,0,17,i+1);	//»Ø´ð³·Ïú
//							CRT_alarm(3,KBll,KBaa,0);                //QD_CRT
							djbc &=0xfd; //djb[16][i]=djbc;
							hjjs[17-1][i+1]=0;
							VZHD--;
//USART1_SendByte(0xa7);
						}
						hjjs[17-1][i+1]=0;
					}
					if((djbc&0x84)==0x84)     //GZHF
					{	if((dat&0x01)==0x01)
						{	HJalarm(21,0,17,i+1); //fault_repair
							djbc&=0xfb; //djb[16][i]=djbc;
							VZGZ--; gzjs[17-1][i+1]=0;
//						CRT_alarm(21,ll+1,aa,0);
							if((i&0x01)==0) U7_txQD[dll][i/2] &=0xdf;
							else						U7_txQD[dll][i/2] &=0xfd;
//USART1_SendByte(0xa8);
						}
					}						
					if((djbc&0x84)==0x80)					//GZ
					{	if((dat&0x01)==0x00) gzjs[17-1][i+1]++; else gzjs[17-1][i+1]=0;
						if(gzjs[17-1][i+1]>5) 		//gzcycle
						{	
//USART1_SendByte(0xa9);
							HJalarm(16,0,17,i+1);	//fault
							djbc|=0x04; //djb[16][i]=djbc;
							VZGZ++; gzjs[17-1][i+1]=0;
							if((i&0x01)==0) U7_txQD[2][i/2] |=0x20;
							else						U7_txQD[2][i/2] |=0x02;
						}
					}
				}				
				djb[16][i+1]=djbc;
			}  	//for(i=0;i<32;i++) end
		U7_recOK[U7_mode]=0;
	}				//if(Uart7_recOK==1)	//commu end
	if((U7_cmdbz[dll]&0x12)==0x12) 		//Zhiqi out cmd=12 cmd|=04_FW cmd|=08_Dengj cmd|=04_JinZhi
		{	//Fa F0(0) 12 Q01 ---Q3031(17) F2(18)  Q0:ok_0000 GZdeng_0010 SouQi_0100
		dat=0xf0;			// ·¢begin
		UART7->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);
		dat=U7_cmdbz[dll];			// ·¢Ö±Æô¡¡cmd¡¡12
		UART7->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);
		for(i=0;i<16;i++)
		{	dat=U7_txQD[dll][i];			// ·¢QD
//USART1_SendByte(dat);
			UART7->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);
		}
		dat=0xf2;			// ·¢end
		UART7->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);

		U7_Lastcmd[dll]=0x12;
		U7_LastrecLen[dll]=18;
		U7_mode =dll;        //2 at INT7 store to U7_rxKB[2][i] 
	}
// orther cmd
	Uart7_recZZ=0;	U7_LastLL=dll;	U7_cmdbz[dll]=0x12;
}


void Usart_loop2ZXP(u8 dll)			//this  dll==0 ZongXianPan
{	u8 i,j,dat,datb,KBnum,djbc,conv,KBll,KBaa;
	u16 KBconvadr;
//	GPIOI->ODR=0;  //dll;
//	Lpscan_on();Delay16s(16,1);
//	Ledscan_off();
	if(U2_recOK==1)	//communicata NO error
	{	
		if(U2_Lastcmd==2)	//cmd=2 Loop
		{	
			for(i=0;i<16;i++)
			{	dat=U2_rxKB[i];
				datb=U2_rxKBbak[i];
				for(j=0;j<8;j++)
				{	conv=bconv[j];
					if(((datb &conv)==0)&&((dat &conv)!=0)) //QD (i*8+j+1)key
					{	KBnum =i*8+j;
						KBconvadr=KBnumadr[dll][KBnum];
						KBll=KBconvadr>>8; KBaa=KBconvadr &0x00ff;
						djbc=djb[KBll-1][KBaa];  
						if((djbc&0x20)==0x00)	//
						{	HJalarm(3,0,KBll,KBaa);							//
							U7_QDHDcmd(3,(KBll-1)*256+KBaa);
							LedLD_on();KYXLD_on();KZYX_on();djbc |=0x20; //linshi_diandeng
//							CRT_alarm(3,KBll,KBaa,0);                //QD_CRT
							djbc |=0x20; lpxq_bz[KBll-1]=0x02;
//	USART1_SendByte(0xb3);
						}
						djb[KBll-1][KBaa]=djbc;
					}	
					if(((datb &conv)!=0)&&((dat &conv)==0)) //TZ (i*8+j+1)key
					{	KBnum =i*8+j;
						KBconvadr=KBnumadr[dll][KBnum];
						KBll=KBconvadr>>8; KBaa=KBconvadr &0x00ff;
						djbc=djb[KBll-1][KBaa];  
						if((djbc&0x20)==0x20)	//
						{	HJalarm(4,0,KBll,KBaa);							//
							U7_QDHDcmd(4,(KBll-1)*256+KBaa);
							djbc &=0xdf; //linshi_diandeng
//							CRT_alarm(4,KBll,KBaa,0);                //QD_CRT
							djbc &=0xdf; lpxq_bz[KBll-1]=0x02; 
//USART1_SendByte(0xfd);
						}
						djb[KBll-1][KBaa]=djbc;
					}	
				}	
			}
			for(i=0;i<16;i++)
			{	U2_rxKBbak[i]=U2_rxKB[i];}
		}
		U2_recOK =0;
	}
//	U7_mode =0;
	if(U2_cmdbz[dll]==0x02) 		//out cmd=2
	{	
		dat=0xf0;			// ·¢begin
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		dat=0x02;			// ·¢cmd
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		for(i=0;i<16;i++)
		{	dat=U2_txQD[dll][i];			// ·¢QD
			USART2->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		}
		for(i=0;i<16;i++)
		{	dat=U2_txHD[dll][i];			// ·¢HD
			USART2->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		}
		dat=0xf2;			// ·¢end
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		U2_Lastcmd=0x02;	U2_LastrecLen=18;
	}
	if(U2_cmdbz[dll]==0x06) 			//Reset cmd=6 at ResetC
	{	
		dat=0xf0;			// ·¢begin
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		dat=0x06;			// ·¢cmd
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		dat=0xf2;			// ·¢end
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		U2_Lastcmd =0x06; U2_LastrecLen=3;
	}
	if(U2_cmdbz[dll]==0x07) 			//Dengjian cmd=7
	{	
		dat=0xf0;			// ·¢begin
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		dat=0x07;			// ·¢cmd
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		dat=0xf2;			// ·¢end
		USART2->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		U2_Lastcmd =0x07; U2_LastrecLen =18;
	}
// orther cmd
	U2_recZZ=0;	U7_cmdbz[dll]=0x02; //U2_LastLL=dll;
}
void U7_QDHDcmd(u8 cmd,u16 LPllaa)	//to used Uart2 cmd:3_QD 4_TZ 6_HD 7_HDQX Loop LL AA => 
{	u8 i,j,k,m;
	for(j=0;j<=1;j++)
	{	for(i=0;i<128;i++)
		{ if(KBnumadr[j][i]==LPllaa+0x100) //(LPll+1)x100+LPaa
			{	k=i/8;m=0x01<<(i%8);
				if(cmd==3) {	U2_txQD[j][k] |= m;	}
				if(cmd==4) {	U2_txQD[j][k] &= ~(m);	}
				if(cmd==6) {	U2_txHD[j][k] |= m;	}
				if(cmd==7) {	U2_txHD[j][k] &= ~(m);	}
			}
		}
	}
}

void Usart_loop6(void)			//QuYu_JiZong
{	u8 i,dat,QYdjbc,dqy,dalm,dll,daa;
//,dalm,dll,daa;u8 buf[4],dcmd,dnam,dflo,dzon,drom1,drom2;
//	u32 waddr;
//USART1_SendByte(dat); Delay16s(16,2);USART1_SendByte(i);Delay16s(16,2);
	
	if(ut6_jz==0)  //is JiZong
	{
//USART1_SendByte(0x41);Delay16s(16,2);
//		ut6_cmd=0x6d;
		Uart6_RecDat=Uart6_recBuf[0];
		if((Uart6_RecDat==0x85)||(Uart6_RecDat==0xfd))  //communicata OK
		{	if((QuYu_GZ_count&0x80)==0x80)
			{	//HJalarm(21,0,17,197); VZGZ--;	//fault_repair
				QuYu_GZ_count=0;
			}
		}	
		else						//communicata error
		{	//if((Boma&0x40)==0x00)		//Boma_7:CRT_and_QuYu_GZ_control
			{	if((QuYu_GZ_count&0x80)==0x00)
				{	QuYu_GZ_count+=1;
					if((QuYu_GZ_count&0x7f)>=30)
					{	//HJalarm(16,0,17,197); VZGZ++; //fault
						QuYu_GZ_count |=0x80;
					}
				}	
			}
		}	
//		if(Uart6_RecDat==0x85){	USART1_SendByte(0x86);		}
		if(Uart6_RecDat==0xfd) //return ALM LL AA ->QuYu_rxbuf
		{	ur6_num=Uart6_recBuf[2]; ur6_cmd=Uart6_recBuf[3];
			ur6_ll=Uart6_recBuf[4];	 ur6_aa=Uart6_recBuf[5];
			
			if(ur6_cmd ==0x42)		//HJ
			{	QYdjbc=((QYdjb[ur6_ll-1][ur6_aa]) & (bconv[ur6_num&0x07]));
				if(QYdjbc==0)
				{		HJalarm(1,ur6_num,ur6_ll,ur6_aa);
						VZHJ++;	haveHJ=1; LedHJ_on();
						//liandongBC(ll+1,aa);
						CRT_alarm(1,ur6_ll,ur6_aa,ur6_num+1);	//¼¯ÖÐ»ú´«ËÍÇøÓò»ú»ð¾¯µ½CRT
						QYdjb[ur6_ll-1][ur6_aa] |=(bconv[ur6_num&0x07]);
				}
			}
			if(ur6_cmd ==0x67)		//GZ
			{	QYdjbc=((QYdjbgz[ur6_ll-1][ur6_aa]) & (bconv[ur6_num&0x07]));
				if(QYdjbc==0)
				{		HJalarm(16,ur6_num,ur6_ll,ur6_aa);
						VZGZ++;	LedGZ_on();
						//liandongBC(ll+1,aa);
						CRT_alarm(16,ur6_ll,ur6_aa,ur6_num+1);	//¼¯ÖÐ»ú´«ËÍÇøÓò»úGZµ½CRT
						QYdjbgz[ur6_ll-1][ur6_aa] |=(bconv[ur6_num&0x07]);
				}
			}
			if(ur6_cmd ==0x69)		//GZHF
			{	QYdjbc=((QYdjbgz[ur6_ll-1][ur6_aa]) & (bconv[ur6_num&0x07]));
				if(QYdjbc!=0)
				{		HJalarm(21,ur6_num,ur6_ll,ur6_aa);
						VZGZ--;	 LedHJ_on();
						//liandongBC(ll+1,aa);
						CRT_alarm(21,ur6_ll,ur6_aa,ur6_num+1);	//¼¯ÖÐ»ú´«ËÍÇøÓò»úHFµ½CRT
						QYdjbgz[ur6_ll-1][ur6_aa] &=(~(bconv[ur6_num&0x07]));
				}
			}
		}
//USART1_SendByte(ur6_cmd);Delay16s(16,2);USART1_SendByte(ur6_num);Delay16s(16,2);			

		dalm =0x6d; dll=0; daa=0;
		if(QYalm_num !=0)
		{	for(i=0;i<=32;i++)		//Up cmd Pros
			{ if(QuYu_txbuf[i][0]!=0)
				{	dalm=QuYu_txbuf[i][0];dll=QuYu_txbuf[i][1];daa=QuYu_txbuf[i][2];
					dqy=QuYu_txbuf[i][3];	
					QuYu_txbuf[i][0]=0;
					QYalm_num --;
					break;
				}
			}
		}
//USART1_SendByte(dalm);Delay16s(16,2);			
		Ur485_2=1; //!!!! Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
		if(dalm==0x6d)		//¡¡Ë³ÐòÑ²¼ì
		{	ut6_num+=1; if(ut6_num > ut6_max) ut6_num=1; QuYustrb[4]=0;QuYustrb[5]=0;
			QuYustrb[2]=ut6_num; QuYustrb[3]=0x6d;
		}
		else								//¡¡Æô¶¯ÇøÓò»ú
		{	QuYustrb[2]=0; 	QuYustrb[3]=dalm;	QuYustrb[4]=dll;QuYustrb[5]=daa;
		}
		QuYustrb[0]=0xfd; QuYustrb[1]=0x7a;
		for(i=0;i<13;i++)			// from 1 to LEN
		{	dat=QuYustrb[i];			//·¢ËÍ JiZong Êý¾Ý
//USART1_SendByte(dat);	
			USART6->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
		}
		Delay16s(60,200); //400us
		Ur485_2=0;
//		ut6_cmd=0x6d;
	}		//if(ut6_jz==0)  //is JiZong end
//USART1_SendByte(dcmd);Delay16s(16,2);	
	
	if(ut6_jz==1)  //is QuYu
	{
//USART1_SendByte(0x42);Delay16s(16,2);USART1_SendByte(Uart6_recOK);Delay16s(16,2);
		if(Uart6_recOK==2)  //rec fd 7a---fe ok
		{	Uart6_RecDat=Uart6_recBuf[0];
			ur6_num=Uart6_recBuf[2]; ur6_cmd=Uart6_recBuf[3];
			ur6_ll=Uart6_recBuf[4];	 ur6_aa=Uart6_recBuf[5];
//USART1_SendByte(ur6_cmd);Delay16s(16,2);Delay16s(16,2);
//USART1_SendByte(ur6_num);Delay16s(16,2);Delay16s(16,2);
//USART1_SendByte(ut6_num);Delay16s(16,2);Delay16s(16,2);
			if((Uart6_RecDat==0xfd))		// && (ur6_num==ut6_num)) //ur6_num 01 ut6_num 00
			{	switch(ur6_cmd)			//Down cmd Pros
				{	case 0x63:				//QiDong
					{	HJalarm(3,ur6_num,ur6_ll,ur6_aa);
//USART1_SendByte(0x44);
						//						haveHJ=1; liandongBC(ll+1,aa);
//						CRT_alarm(1,ll+1,aa,ur6_num);	//;QuYu NO CRT
						QYdjb[ur6_ll-1][ur6_aa] |=(bconv[ur6_num&0x07]);
						break;
					}
					case 0x6D:		//	XunJian
					{	//if(bz_Non) return 85 else ret HJ_QD
//USART1_SendByte(QYalm_num);Delay16s(16,2);
						if(QYalm_num !=0)			//ÇøÓò»úans HJ_0x41-- Non_0x85
						{	
							for(i=0;i<=32;i++)		//Up cmd Pros
							{ if(QuYu_txbuf[i][0]!=0)
								{	dalm=QuYu_txbuf[i][0];dll=QuYu_txbuf[i][1];daa=QuYu_txbuf[i][2];
//								dqy=QuYu_txbuf[i][3];
									QuYu_txbuf[i][0]=0;
									QYalm_num --;
									break;
								}
							}
							QuYustrb[0]=0xfd; QuYustrb[1]=0x7a; 
							QuYustrb[2]=ut6_max; //!!! 
							QuYustrb[3]=dalm; 
							QuYustrb[4]=dll;QuYustrb[5]=daa;QuYustrb[11]=ut6_max;
//							Ur485_2=1; //!!!! Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
							for(i=0;i<13;i++)			// from 1 to LEN
							{	dat=QuYustrb[i];			//·¢ËÍ JiZong Êý¾Ý
//USART1_SendByte(dat);				
								USART6->DR = (dat & (uint16_t)0x01FF);
								while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
							}
							Delay16s(60,200); //400us
//							r485_2=0;
						}
						break;
					}
					case 0x6f:				//FuWei
					{	
//USART1_SendByte(0xcd);
						ResetC();
						break;
					}
				}
			}	
		}
//USART1_SendByte(QYalm_num);
//		if(QYalm_num !=0)			//ÇøÓò»úans HJ_0x41-- Non_0x85
//		{	
//			for(i=0;i<=32;i++)		//Up cmd Pros
//			{ if(QuYu_txbuf[i][0]!=0)
//				{	dalm=QuYu_txbuf[i][0];dll=QuYu_txbuf[i][1];daa=QuYu_txbuf[i][2];
////					dqy=QuYu_txbuf[i][3];
//					QuYu_txbuf[i][0]=0;
//					QYalm_num --;
//					break;
//				}
//			}
//			QuYustrb[0]=0xfd; QuYustrb[1]=0x7a; 
//			QuYustrb[2]=ut6_max; //!!! 
//			QuYustrb[3]=dalm; 
//			QuYustrb[4]=dll;QuYustrb[5]=daa;QuYustrb[11]=ut6_max;
//			Ur485_2=1; //!!!! Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
//			for(i=0;i<13;i++)			// from 1 to LEN
//			{	dat=QuYustrb[i];			//·¢ËÍ JiZong Êý¾Ý
////USART1_SendByte(dat);				
//				USART6->DR = (dat & (uint16_t)0x01FF);
//				while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
//			}
//			Delay16s(60,200); //400us
//			Ur485_2=0;
//		}
		Uart6_recOK=0;
	}		//if(ut6_jz==1)  //is QuYu end
}		//Usart_loop6()	//QuYu_JiZong  END

void Usart_loop3(u8 dll)
{	u8 i,j,T_len,dat;
//Test_12=1;

	GPIOI->ODR=dll;
	Lpscan_on();Delay16s(16,1);
	Ledscan_off();
//USART1_SendByte(0xa0+dll+Uart3_recOK*0x10);
	if(Uart3_recOK==1)	//communicata NO error
	{	
		if((lpstrb[Uart3_LastLL][1]==1)||(lpstrb[Uart3_LastLL][1]==2))	//cmd=1,2 fire
		{	if(Uart3_LastLL<=Lp_max-1)
			{ for(i=0;i<201;i++) {	lpxz_val[Uart3_LastLL][i]=Uart3_recBuf[i+1];} 
				for(i=0;i<25;i++)
				{ dat=Uart3_recBuf[i+202];
					if(dat&0x01)     lpxz_bit[Uart3_LastLL][i*8+1]|=1;else lpxz_bit[Uart3_LastLL][i*8+1]&=0xfe;
					if((dat>>1)&0x01)lpxz_bit[Uart3_LastLL][i*8+2]|=1;else lpxz_bit[Uart3_LastLL][i*8+2]&=0xfe;
					if((dat>>2)&0x01)lpxz_bit[Uart3_LastLL][i*8+3]|=1;else lpxz_bit[Uart3_LastLL][i*8+3]&=0xfe;
					if((dat>>3)&0x01)lpxz_bit[Uart3_LastLL][i*8+4]|=1;else lpxz_bit[Uart3_LastLL][i*8+4]&=0xfe;
					if((dat>>4)&0x01)lpxz_bit[Uart3_LastLL][i*8+5]|=1;else lpxz_bit[Uart3_LastLL][i*8+5]&=0xfe;
					if((dat>>5)&0x01)lpxz_bit[Uart3_LastLL][i*8+6]|=1;else lpxz_bit[Uart3_LastLL][i*8+6]&=0xfe;
					if((dat>>6)&0x01)lpxz_bit[Uart3_LastLL][i*8+7]|=1;else lpxz_bit[Uart3_LastLL][i*8+7]&=0xfe;
					if((dat>>7)&0x01)lpxz_bit[Uart3_LastLL][i*8+8]|=1;else lpxz_bit[Uart3_LastLL][i*8+8]&=0xfe;
				}
			}
			else
			{	for(i=0;i<201;i++)
				{	lpxz_val[Uart3_LastLL][i]=0;
					lpxz_bit[Uart3_LastLL][i]=0;
				} 
			}	
		}
		Uart3_recOK=0;
	}
	if(lpxq_bz[dll]==0x02)		//out cmd=2
	{
		for(j=0;j<25;j++)
		{ dat=0;
			for(i=0;i<8;i++)
			{ if((djb[dll][j*8+i+1]&0x20)!=0) dat |=bconv[i]; 
			}
			lpstrb[dll][j+2]=dat;
		}
		T_len=28;lpstrb[dll][0]=0xf0;lpstrb[dll][1]=0x02;
		lpstrb[dll][27]=0xf2;	Uart3_LastCmd=2; Uart3_LastrecLen=228;
	}
	if(lpxq_bz[dll]==0x01) 			//out cmd=1
	{	
		T_len=3;lpstrb[dll][0]=0xf0;lpstrb[dll][1]=0x01; 
		lpstrb[dll][2]=0xf2;   Uart3_LastCmd=1; Uart3_LastrecLen=228;
	}
//	if(lpxq_bz[dll]==0x06) 			//Reset cmd=6 at ResetC
//	{	
//		T_len=3;lpstrb[dll][0]=0xf0;lpstrb[dll][1]=0x06; 
//		lpstrb[dll][2]=0xf2;   Uart3_LastCmd=1; Uart3_LastrecLen=228;
//	}
	Uart3_recZZ=0;	Uart3_LastLL=dll;
// orther cmd

	for(i=0;i<T_len;i++)			// from 1 to LEN
	{	dat=lpstrb[dll][i];			//·¢ËÍ»ØÂ·Êý¾Ý
		USART3->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
//USART1_SendByte(dat);
//      UART7->DR = (dat & (uint16_t)0x01FF);
//      while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);  
	}
//Test_12=0;
}
 void Usart_loop(u8 zz,u8 dll)	//yuan
 {
   u8 i,dat;
   prlong3=0;prlong=0;
	 GPIOI->ODR=dll;
	 Lpscan_on();Delay16s(26,1);
	 Ledscan_off();
//USART1_SendByte(0x80+dll);				
	 for(i=0;i<4;i++)
	   {dat=strblp[zz][i];
			//·¢ËÍ»ØÂ·Êý¾Ý
	    USART3->DR = (dat & (uint16_t)0x01FF);
      while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
      UART7->DR = (dat & (uint16_t)0x01FF);
      while (USART_GetFlagStatus(UART7,USART_FLAG_TXE) == RESET);  
      if(dat==0xf2)break;			 
	   } 
 }
void CRT_alarm(u8 alarm,u8 ll,u8 aa,u8 quyu)  //HJ_GZ_HD at Pross_HJ, QD at HJalarm
{	u8 i;                        //this store,Usart_loop7() transmit txbuf[20][3]
	for(i=0;i<=200;i++)
	{	if(CRT_txbuf[i][0]==0)
		{	CRT_txbuf[i][0]=alarm;CRT_txbuf[i][1]=ll;CRT_txbuf[i][2]=aa;CRT_txbuf[i][3]=quyu;
			ut6_cmd=alarm;
//	USART1_SendByte(0x30+i);Delay16s(16,2);	USART1_SendByte(alarm);Delay16s(16,2);			
			break;
		}
	}
}
void CRT_loop(void)  //0x6d loop
{	u8 i,dalm,dcmd,dll,daa,dquyu,dat; 
//	Ur485_2=1;  Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
//	if((Boma&0x04)==0) { return; }
	dcmd=0x6d;
	for(i=0;i<=200;i++)		//Up cmd Pros
	{ if(CRT_txbuf[i][0]!=0)
		{	dalm=CRT_txbuf[i][0];dll=CRT_txbuf[i][1];
			daa=CRT_txbuf[i][2];dquyu=CRT_txbuf[i][3];
			switch(dalm)
			{	//case 0: {	dcmd=0x42;}	break;		//SJ
				case 1: {	dcmd=0x42;}	break;		//HJ
				case 3: {	dcmd=0x63;}	break;		//QD
				case 4: {	dcmd=0x65;}	break;		//TZ
				case 6: {	dcmd=0x61;}	break;		//HD
				case 7: {	dcmd=0x6b;}	break;		//HDQX
				case 16:{	dcmd=0x67;}	break;		//GZ
				case 21:{	dcmd=0x69;}	break;		//GZHF
				case 32:{	dcmd=0x51;}	break;		//PB
				case 33:{	dcmd=0x53;}	break;		//PBCX
//				case 66:{	dcmd=0x;}	break;		//ZuDian
				default:{	dcmd=0x6d;}
			}
//USART1_SendByte(dalm);
			break;
		}	
	}
	if(dcmd!=0x6d)	//stop CRT_Loop 
	{
		CRT_txbuf[i][0] =0;
		CRTstrb[0]=0xfd;CRTstrb[1]=0x7f;CRTstrb[2]=dquyu;CRTstrb[3]=dcmd;CRTstrb[4]=dll;CRTstrb[5]=daa;
		CRTstrb[6]=0;CRTstrb[7]=0;CRTstrb[8]=0;CRTstrb[9]=0;CRTstrb[10]=0;CRTstrb[12]=0xfe;
		CRTstrb[11]=CRTstrb[0]+CRTstrb[1]+CRTstrb[2]+CRTstrb[3]+CRTstrb[4]+CRTstrb[5]+CRTstrb[6]+CRTstrb[7]+CRTstrb[8]+CRTstrb[9]+CRTstrb[10];
		//???? [11]?
		Ur485_2=1; 
		Delay16s(60,20); //40us
		for(i=0;i<13;i++)			// from 1 to LEN
		{	dat=CRTstrb[i];			//?? JiZong ??
			USART6->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
		}
//USART1_SendByte(dcmd);
		Delay16s(60,200); //400us
		Ur485_2=0;
	}
}		

void CRT_Fuwei(void)  //0x6f fw
{	u8 i,dll,daa,dat,dcmd;  //dalm,
//	Ur485_2=1;  Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
	dcmd=0x6f;
	CRTstrb[0]=0xfd;CRTstrb[1]=0x7f;CRTstrb[2]=0x00;CRTstrb[3]=dcmd;CRTstrb[4]=dll;CRTstrb[5]=daa;
	CRTstrb[6]=0;CRTstrb[7]=0;CRTstrb[8]=0;CRTstrb[9]=0;CRTstrb[10]=0;CRTstrb[12]=0xfe;
	CRTstrb[11]=CRTstrb[0]+CRTstrb[1]+CRTstrb[2]+CRTstrb[3]+CRTstrb[4]+CRTstrb[5]+CRTstrb[6]+CRTstrb[7]+CRTstrb[8]+CRTstrb[9]+CRTstrb[10];
		//Ð£ÑéºÍÔÚ [11]¡[
	Ur485_2=1; 
	Delay16s(60,20); //40us
	for(i=0;i<13;i++)			// from 1 to LEN
	{	dat=CRTstrb[i];			//·¢ËÍ JiZong Êý¾Ý
		USART6->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
	}
//USART1_SendByte(dcmd);
	Delay16s(60,200); //400us
	Ur485_2=0;
}

void QuYu_alarm(u8 alarm,u8 ll,u8 aa,u8 quyu)  //HJ_GZ_HD at Pross_HJ, QD at HJalarm
{	u8 i;                        //this store,Usart_loop7() transmit txbuf[20][3]
	for(i=0;i<=32;i++)
	{	if(QuYu_txbuf[i][0]==0)
		{	QuYu_txbuf[i][0]=alarm; QuYu_txbuf[i][1]=ll; QuYu_txbuf[i][2]=aa;
			QuYu_txbuf[i][3]=quyu;
			QYalm_num +=1;
			//			ut6_cmd=alarm;
//USART1_SendByte(QuYu_txbuf[0][0]);Delay16s(16,2);	
//USART1_SendByte(QuYu_txbuf[0][1]);Delay16s(16,2);	
//USART1_SendByte(QuYu_txbuf[0][2]);Delay16s(16,2);	
//USART1_SendByte(QuYu_txbuf[0][3]);Delay16s(16,2);	
			break;
		}
	}
}

void QuYu_Fuwei(void)  //0x6f fw
{	u8 i,dll,daa,dat,dcmd;  //dalm,
//	Ur485_2=1;  Ur485_1=1; all chang Ur485_2_used_CRT,Ur485_1_used_QuYu
	QYalm_num=0;
	dcmd=0x6f;
	QuYustrb[0]=0xfd;QuYustrb[1]=0x7a;QuYustrb[2]=0x01;QuYustrb[3]=dcmd;
	QuYustrb[4]=dll;QuYustrb[5]=daa;QuYustrb[6]=0;QuYustrb[7]=0;QuYustrb[8]=0;
	QuYustrb[9]=0;QuYustrb[10]=0;QuYustrb[12]=0xfe;
//	CRTstrb[11]=CRTstrb[0]+CRTstrb[1]+CRTstrb[2]+CRTstrb[3]+CRTstrb[4]+CRTstrb[5]+CRTstrb[6]+CRTstrb[7]+CRTstrb[8]+CRTstrb[9]+CRTstrb[10];
		//Ð£ÑéºÍÔÚ [11]¡[
	Ur485_2=1; 
	Delay16s(60,20); //40us
	for(i=0;i<13;i++)			// from 1 to LEN
	{	dat=QuYustrb[i];			//·¢ËÍ JiZong Êý¾Ý
		USART6->DR = (dat & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
	}
//USART1_SendByte(dcmd);
	Delay16s(60,200); //400us
	Ur485_2=0;
}		
 

void Cengx_loop(void)
{	u8 i,j,dll,dat,baki;	

	for(i=0;i<=19;i++)
	{ if((CengX_txbuf[i][11]!=20)) //ll=20 is spc
		{	for(j=0;j<12;j++) CengXstrb[j]=CengX_txbuf[i][j];
			dll=CengXstrb[11]&0x1f;
			baki=i;
//for(j=0;j<12;j++) {USART1_SendByte(CengX_txbuf[i][j]);Delay16s(16,2);}
			break;
		}
	}
  //USART1_SendByte(0xcb);USART1_SendByte(dll);
//USART1_SendByte(lpzz);Delay16s(16,1);
	if(dll==(lpzz+1))  //dll=is Current_LL
	{	
//USART1_SendByte(dll);Delay16s(16,1);
		TIM_ITConfig(TIM6,TIM_IT_Update,DISABLE);  	
		GPIOI->ODR=dll-1;
		Lpscan_on();Delay16s(16,1);
		Ledscan_off();
		for(i=0;i<11;i++)			
		{	dat=CengXstrb[i];			//
			USART3->DR = (dat & (uint16_t)0x01FF);
			while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
//USART1_SendByte(dat);
		}
		CengXstrb[11]=0;
		CengX_txbuf[baki][11]=20;  //ll=20 is spc
		Delay1ms(500);TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
 	}
}	
void CengX_alarm(u8 alarm,u8 dll,u8 daa)  //this store,CengX transmit CengX_txbuf[20][12]
{	                          //fo 09 CXadr alm+ll aa Mh+D Ml+H Min  00   FD  f2 CXll 
	u32 waddr;								//f0 09 CXadr alm+ll aa nam  flo  zon rom1 rom2 f2 CXll
	u8 i,CXll,CXaa;		  			// 0  1   2     3    4   5    6    7   8    9   10   11  
	u8 ri,shi,fen,buf[26];
	for(i=0;i<=19;i++)
	{	if(CengX_txbuf[i][11]==20)
		{	waddr=0xa0000+(dll-1)*6400+(daa-1)*32;
			SPI_Flash_Read(buf,waddr,26); 
//			if(buf[20]<0x80) {drom1=buf[20]; drom2=buf[21];}
//			else {drom1=0; drom2=1;}
			CXll=buf[22];CXaa=buf[23];
//			if(dll==0) 					//80_ZiJian c0_fuwei
//			{ ri=0; shi=0; fen=0;	}
//			else								// CX_alarm
			{	fen=ctime[0];
				shi=ctime[1]+((ctime[3]&0x03)<<5);
				ri=ctime[2]+((ctime[3]&0x0c)<<3);
			}	
			CengX_txbuf[i][0]=0xf0; CengX_txbuf[i][1]=0x09;  CengX_txbuf[i][2]=CXaa;
			CengX_txbuf[i][4]=daa;   CengX_txbuf[i][5]=ri;
			CengX_txbuf[i][6]=shi;  CengX_txbuf[i][7]=fen;   CengX_txbuf[i][8]=0;
			CengX_txbuf[i][9]=0xfd; CengX_txbuf[i][10]=0xf2; CengX_txbuf[i][11]=CXll;
			CengX_txbuf[i][3]=dll+0x40;  //40;HJ 00:time FW:-_-
//USART1_SendByte(CXll);USART1_SendByte(CXaa);USART1_SendByte(dll);USART1_SendByte(daa);
//USART1_SendByte(dnam);USART1_SendByte(dflo);USART1_SendByte(dzon);			
//USART1_SendByte(i);
//for(j=0;j<12;j++) {USART1_SendByte(CengX_txbuf[i][j]);Delay16s(16,2);}
			break;
		}
	}
}		

//KBval[k][i]
u16 GBkeyconv(u8 ll,u8 aa)
{ u8 buf[4];u32 waddr; u16 dat;
	waddr=0x9c000+(ll-1)*256+aa*2;
	SPI_Flash_Read(buf,waddr,2);
	dat=(buf[0]*256+buf[1]);
//USART1_SendByte(dat>>8);Delay16s(16,2);USART1_SendByte(dat&0x00ff);Delay16s(16,2);
	return(dat);

//  keyll=0;keyaa=0;
//  fp=fopen("SD2200.GB3","rb");
//  if (fp==NULL) { printf("\16[{-21|460(15)1 SD2200.GB3???  }]"); return; }
//  fseek (fp,(ll-1)*256+(aa-1)*2, SEEK_SET);
//  fread (&keyll,1,1,fp);  fread (&keyaa,1,1,fp);
//  fclose(fp);
}
void GBconv_key(u8 ll,u8 aa)
{ //u8 ml,ma; u16 i;
//  keyll=0;keyaa=0;
//  fp=fopen("SD2200.GB3","rb");
//  if (fp==NULL) { printf("\16[{-21|460(15)1 SD2200.GB3???  }]"); return; }
//  for(i=0;i<2048;i++)
//  { fseek (fp,i*2,SEEK_SET); fread (&ml,1,1,fp); fread (&ma,1,1,fp);
//    if((ml==ll)&&(ma==aa)) { keyll=i/128+1;keyaa=i%128+1; break; }
//  }
//  fclose(fp);
  return;
}

void moni_disp(){
	u16 monix0=24,moniy1=50;
	u8 y,x,adr; u16 disp_val;
	
	if((moniLL>0)&&(moniLL<=16)){
		Dispsz16(moniLL,2,moniy1-24,monix0,DarkRed,White);		 												//moniLL
		for(y=0;y<15;y++)Dispsz16(adr+y*15,3,moniy1+y*24,monix0,DarkRed,White);		 //Y_first_val
		for(x=0;x<15;x++)Dispsz16(adr+x,2,moniy1-24,monix0+40+40*x,DarkRed,White); //X_fiest_val
		for(y=0;y<15;y++){
			for(x=0;x<15;x++){
				if((y*15+x)>200) break;
				disp_val=lpxz_val[moniLL-1][y*15+x]+(lpxz_bit[moniLL-1][y*15+x])*1000;
				if((djb[moniLL-1][y*15+x]&0x80)==0x80) {disp_val+=2000;}
				if((djb[moniLL-1][y*15+x]&0xc0)==0xc0) {disp_val+=4000;}
				if(disp_val==0)
					Dispsz16(disp_val,4,moniy1+y*24,monix0+32+x*40,Magenta,DarkGrey);
				else
					Dispsz16(disp_val,4,moniy1+y*24,monix0+32+x*40,Magenta,White);
			}
		}
	}
}


void Disprom(u32 addr,u8 dll,u8 daa,u16 curv,u16 curh,u16 bColor,u16 fColor)
{ u8 i,k,fsz[24]; u16 xx;
  u32 waddr;
  if(addr==0xa0000){k=8;waddr=addr+(dll-1)*6400+(daa-1)*32+6;} 
//	if(addr==0xd0000){k=10;waddr=addr+(daa-1)*32+6;}             
  if(addr==0x9f000){k=4;waddr=addr+daa*8;}  
  if(addr==0x64000){k=2;waddr=addr+daa*8;}   
  if(addr==0x9e000){k=3;waddr=addr+daa*6;} 
//USART1_SendByte(addr>>12); Delay16s(16,2);  
  SPI_Flash_Read(fsz,waddr,k*2);  
  for(i=0;i<k;i++)
	{	//if(fsz[2*i]<0xa0||fsz[2*i]>0xf0)	//ºº×Ö´®ÖÐ¿É´øÓÐ×Ö·ûÀàÐÍ
		//{	if(fsz[2*i]<0xa0)Dispzf(fsz[2*i],curv,curh,bColor,fColor);	
		//	if(fsz[2*i+1]<=0xa0)Dispzf(fsz[2*i+1],curv,curh+8,bColor,fColor);
		//}
		//else
		xx=(fsz[2*i]<<8)+fsz[2*i+1];
		Disphzs24(xx,curv,curh,bColor,fColor);
		curh+=24;
	}
	LCD_DrawSquareBox(1,1,24,24,Blue);       //Draw left up 
}

void Disptime(u8 *p,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor)
{u8 dcode;															//adjust No Year
 dcode=*p;
 curh =curh-18;		// <-Disp 16
// Dispsz16(dcode,2,curv+6,curh,bColor,fColor);
// Dispzf('/',curv+6,curh+16,bColor,fColor);//yy/mm/dd
 p--;
 dcode=*p;Dispsz16(dcode,2,curv+6,curh+16+8,bColor,fColor);
	        Dispzf('/',curv+6,curh+16+8+16,bColor,fColor);
 p--;
 dcode=*p;Dispsz16(dcode,2,curv+6,curh+16+8+16+8,bColor,fColor);
 p--;
 dcode=*p;Dispsz16(dcode,2,curv+6,curh+16+8+16+8+20,bColor,fColor);
	        Dispzf(':',curv+6,curh+16+8+16+8+20+16,bColor,fColor);//hh:mm
 p--;
 dcode=*p;Dispsz16(dcode,2,curv+6,curh+16+8+16+8+20+16+8,bColor,fColor);
}


void DispTimeUP(void)
{
 Dispsz24(ctime[4]/10,1,TimeUP_y,TimeUP_x,Blue,Cyan);
// Dispzf('/',TimeUP_y,TimeUP_x+24,Blue,White);
 Dispsz24(ctime[3],2,TimeUP_y,TimeUP_x+20,Blue,White);
 Dispzf24('/',TimeUP_y,TimeUP_x+50,Blue,White);
 Dispsz24(ctime[2],2,TimeUP_y,TimeUP_x+62,Blue,White);
 Dispsz24((ctime[4]-ctime[4]/10*10),1,TimeUP_y+26,TimeUP_x,Blue,Cyan);
 Dispsz24(ctime[1],2,TimeUP_y+26,TimeUP_x+20,Blue,Magenta);
 Dispzf24(':',TimeUP_y+26,TimeUP_x+50,Blue,Magenta);
 Dispsz24(ctime[0],2,TimeUP_y+26,TimeUP_x+62,Blue,Magenta);
} 
//ÏÔÊ¾¹«Ë¾Ãû
void DispFactor(void)
{	u8 i; u16 color;
//	USART1_SendByte(Tled);
	i=Tled%6;
	switch(i) {
		case 0: color=Green;		break;
		case 1: color=Red;  		break;
		case 2: color=Yellow;		break;
		case 3: color=Cyan;			break;
		case 4: color=Magenta; 	break;
		case 5: color=White;  	break;
	}
	dstr=HZfactor;
	Dispstr(0,4,0,TimeUP_x,Blue,color);
}	
void DispszHEX(u8 dcode,u16 curv,u16 curh,u16 bColor,u16 fColor) 
{
  u8 i,shuw[2];
  shuw[0]=dcode%16;      //¸ßÎ»
  shuw[1]=(dcode/16)%16;
  for(i=0;i<2;i++)
      {if(shuw[i]<10)shuw[i]=0x30+shuw[i];
	     else 		     shuw[i]=0x37+shuw[i];
      }
  Dispzf(shuw[1],curv,curh,bColor,fColor);
  Dispzf(shuw[0],curv,curh+16,bColor,fColor);
}

void Esc(void)
{u8 i;
 MenuC=MenuF=MenuN=0;key=0;
 hjjlfg=ldjlfg=qtjlfg=djfg=glfg=ldfg=djbfg=gzbfg=pbbfg=0;txcsfg=0;
 hjcxfg=gzcxfg=ldcxfg=0;cifg=0;enter=0;Top=0;xhzz=0;opfg=0;
 zero=0;jlend=0;djbaddr=1;  djbcount=0;
 for(i=0;i<12;i++)shu[i]=0;knum=0;com=0;password=0;
 Disptu(0,Magenta,0,0);//Ö÷½çÃæ
 if((hjflag+gzflag+xspbs+ldflag)!=0)
     {if(hjflag)Dispram(0,0,0,0);
			if(hjzz>1)Dispram(1,hjzz-1,0,0);
      if(gzflag!=0)Dispram(16,gzzz-1,0,0);
      if(xspbs!=0)Dispram(32,xspbs-1,0,0);
			if(ldflag!=0)Dispram(2,ldzz-1,0,0); 
//USART1_SendByte(0x33); //Delay16s(16,2);USART1_SendByte(alm);
     }
 return;
 }

	 

void HJalarm(u8 alm,u8 area,u8 dll,u8 daa)
{	u8 hlg=0,i,nam=0,flo=0,zon,buf[4],k;   //almc,alml;
	u16 j;
	u32 waddr;
	MenuN=0;for(i=0;i<12;i++)shu[i]=0;knum=0;
 //ZQbuffer&=0xf8;ZQbuffer|=0x08; 
	Tlcd=0;
//	enter=0;
//USART1_SendByte(0x42); Delay16s(16,2);USART1_SendByte(alm);
//	if(dll<=16&&alm<=1)
//	{	almc=0x42;alml=dll|0x80;  
//			waddr=0xa0000+(dll-1)*6400+(daa)*32+22;
//			SPI_Flash_Read(buf,waddr,2);
//			if(buf[0]==0){if(buf[1]<16&&buf[1]>0)RS485_down(0x7e,buf[1],almc,dll,daa,nam,flo,zon);}//µÚÒ»¸ö²ãÏÔ
//			else{if(buf[0]<10){if(buf[1]<=250&&buf[1]>0)HLCX_down(buf[0],buf[1],alml,daa,nam,flo,zon);}}
//			if(buf[2]==0){if(buf[3]<16&&buf[3]>0)RS485_down(0x7e,buf[3],almc,dll,daa,nam,flo,zon);}//µÚ¶þ¸ö²ãÏÔ
//			else{
//USART1_SendByte(buf[0]); Delay16s(16,2);USART1_SendByte(buf[1]);			
//				if((buf[0]>0) && (buf[1]>0)) {HLCX_down(buf[0],buf[1],alml,daa,nam,flo,zon);}
//	}
//USART1_SendByte(area); Delay16s(16,2);USART1_SendByte(dll); Delay16s(16,2);
	Logit(alm,area,dll,daa); 
		
	if(alm<=1) //0:SHUOJ,1:HJ
	{	if(hjzz!=0)
		{	hlg=hjzz-1;
//			djb[dll-1][daa]|=0x20;							//Smoker LED
			Kyxbz=0; KYXHJ_on();KZYX_on(); 	//Yin Xiang
			hjflag=1;  
			KZSG_on();		//Éù¹â¡¡¸Ä¡¡»ð¾¯¼ÌµçÆ÷
			if(Spalarm==0) 
			{	Spalarm=1;KLCD_on(); LedSG_on();	KZSG_199();	} //Éù¹âÆô¶¯
//			  if(sggz!=0x80){ZQbuffer|=0x20;LEDbuffer[2]|=0x4;}//Éù¹âÆô¶¯
//       ZQbuffer&=0xf8;ZQbuffer|=0x03;LEDbuffer[1]&=0xef;
		}
	}
	if(alm>1&&alm<=7)	//2~7 LD
	{	hlg=ldzz-1;
//		LedLD_on();//×ÜÆô¶¯ÁÁ
//		if(alm==6)LedFQ_on(); //×Ü·´À¡µÆÁÁ
//		LedLD_on();
		KLCD_on(); 
		if((alm==3)||(alm==2)) 
		{ if(dll<=16)	{ VZQD++; }
			if(dll==17)	{ VZZQ++; }
		}
		if(alm==4)		{	if(VZQD>0) VZQD--;  }
		if(alm==3) 
		{	waddr=0xa0000+(dll-1)*6400+(daa-1)*32;
			SPI_Flash_Read(buf,waddr,1);  
			k=buf[0];
			if(k>0x80) k=k-0x40;  //LQL
			if(k==127)		//is SengGuang
			{	
//USART1_SendByte(dll);	Delay16s(16,2);USART1_SendByte(daa);	Delay16s(16,2);
				for(j=1;j<511;j++)
				{	if((SGGB[j]) ==((dll)*256+daa))
					{	SGGB[j] |=0x8000;
						bzSGGB =1;
//SART1_SendByte(SGGB[j]/256);	Delay16s(16,2);
//USART1_SendByte(SGGB[j]&0x00ff);Delay16s(16,2);USART1_SendByte(j);
						break;
					}
				}
			}
		}
		if(dll==17)   //ZQP_Loop17 QD TZ at here. GZ at Usart_loop7ZhiQi(u8 dll)
		{
//USART1_SendByte(alm);
			if(alm==3)
			{
//				U7_txQD[2][zaa] |=zval;	
//USART1_SendByte(zaa);
//USART1_SendByte(U7_txQD[2][zaa]);
			}
			if(alm==4)
			{
//				U7_txQD[2][zaa] &=~(zval);	
//USART1_SendByte(zaa);
//USART1_SendByte(U7_txQD[2][zaa]);
			}
		}
//		if((dll==1)&&(daa==199))
//		{	if(SpalmOK==1)	{KYXLD_on();KZYX_on(); } } //MianBan_SG:HJ_sound
//		else {KYXLD_on();KZYX_on(); }
		Kyxbz=0; KYXLD_on();KZYX_on(); 	//Yin Xiang
//		CRT_alarm(alm,dll,daa,0);                //QD_CRT
		//     if(ldzz!=0){ZQbuffer&=0xf8;ZQbuffer|=0x05;LEDbuffer[1]&=0xef;}
//     else ZQbuffer&=0xf8;
	} 
	if(alm>=16&&alm<=20) //¹ÊÕÏ
  {	if(gzzz!=0)
		{	hlg=gzzz-1;
			LedGZ_on();
			Kyxbz=0; KYXGZ_on();KZYX_on(); 	//Yin Xiang
			ZQbuffer&=0xf8;ZQbuffer|=0x01;KLCD_on();
//			VZGZ++;	
		}
  }
	if(alm==21)
	{	if(gzzz!=0)hlg=gzzz-1;
//		VZGZ--;
//USART1_SendByte(alm+0x30);
	}//¹ÊÕÏ»Ö¸´
	if(alm==32||alm==33)
	{	hlg=xspbs-1; }		//PingBi VZPB++;
	if(alm<=1&&machine>0)RS485_up(0x7a,machine,alm,dll,daa,nam,flo,zon);
	if(CRTxj==1)RS485_down(0x7f,machine,alm,dll,daa,nam,flo,zon);
//USART1_SendByte(alm);Delay16s(16,2);USART1_SendByte(hlg);Delay16s(16,2);
//USART1_SendByte(area); Delay16s(16,2);
	Dispram(alm,hlg,0,area);
	if(printer)Prntjl(alm,area,dll,daa);
//	if((hjflag+gzflag)==0){ZQbuffer&=0xf8;Esc();}
	return;
}

	

void Dispram(u8 alm,u8 hlg,u8 exam,u8 area)
{		u8 time[5],buf[4],nam,flo,zon,i,xll,xarea; //i,
    u16 pos=55,color=Red;          //dloop=0,
		u32 waddr;
		if(exam==1)hjcxfg++;
		if(exam==2)ldcxfg++; 
    if(exam==3)gzcxfg++;
    if(alm<=1)		//is HJ
		{	hjflag=1;
			if(hjcxfg>hlg)return;else hlg-=hjcxfg;
			if(hlg==0){pos=HJtop_y;color=Black;}else {pos=HJtop_y*2;color=Black;}
lpHJ1:
			//ÐÂÔö(½â¾öË¢ÐÂÓÐ²ÐÁô)
			ClearXY(pos,nob_x,26,670,Black);
			xarea=Disp_hj[hlg][3];
			LCD_DrawSquareBox(left_x+1,pos,righ_x-1 ,24,color);       //clear alm_x
			Dispsz16(hlg+1,4,pos,nob_x,color,White);  							//nob
			Disprom(H_alm,0,Disp_hj[hlg][0],pos,alm_x,color,White); //alm
//USART1_SendByte(area);Delay16s(16,2);USART1_SendByte(Disp_hj[hlg][1]);Delay16s(16,2);
			xll=Disp_hj[hlg][1];
			if(xarea!=0) {xll=xll+xarea*20; }
			Dispsz24(xll,2,pos,adr_x,color,White);			//adr_LL
			Dispsz24(Disp_hj[hlg][2],3,pos,adr_x+34,color,White);		//adr_AAA
//			i=xz[Disp_hj[hlg][1]][Disp_hj[hlg][2]];
//			Dispsz16(i,3,pos,val_x+18,color,White);		//adr_VAL
			time[4]=Disp_hj[hlg][4];
			time[3]=(Disp_hj[hlg][5]>>5)*8+(Disp_hj[hlg][6]>>5);
			time[2]=Disp_hj[hlg][5]&0x1f;
			time[1]=Disp_hj[hlg][6]&0x1f;
			time[0]=Disp_hj[hlg][7];
			Disptime(time+4,4,pos,tim_x,color,White);	//time
			if(xarea!=0) {Dispsz16(xarea,2,pos+2,adr_x-18,color,White); } //disp small area
//			Disptime(time+4,4,pos-7,tim_x,color,White);	//time
			if(xarea==0)
			{ waddr=0xa0000+(Disp_hj[hlg][1]-1)*6400+(Disp_hj[hlg][2]-1)*32; }
			else
			{ waddr=0x100000+(xarea-1)*0x20000;
				waddr=waddr+(Disp_hj[hlg][1]-1)*6400+(Disp_hj[hlg][2]-1)*32;
			}
//USART1_SendByte(waddr>>16);Delay16s(16,2);			
			SPI_Flash_Read(buf,waddr,3); 
			nam=buf[0];flo=buf[1];zon=buf[2];
			Disprom(H_nam,0,nam,pos,nam_x,color,White); //dev_name
//USART1_SendByte(nam);Delay16s(16,2);USART1_SendByte(zon);Delay16s(16,2);
//USART1_SendByte(flo);Delay16s(16,2);
			if(flo>128)  																//-Â¥²ã
			{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
				Dispsz24(flo,1,pos,flo_x+12,color,White);
			}
			else
			{	Dispsz24(flo,2,pos,flo_x,color,White);
			}
			Disprom(H_rom,Disp_hj[hlg][1],Disp_hj[hlg][2],pos,rom_x,color,White);//·¿¼äÃû³Æ
			Disprom(H_zon,0,zon,pos,zon_x,color,White); //zon
			if(hlg>0)
			{	hlg--;
				if(hlg==0){	pos=HJtop_y;}
				else 			{ pos +=HJtop_y;}
				if(pos<6*HJtop_y)	goto lpHJ1;
			}			
		}
		/*ÐÂÔöÁª¶¯²»ÏÔÊ¾*/
		if(alm>1&&alm<8&&BZ_CHECK==0)
		{	
			if(ldzz==0)
			{	ldflag=0;
				i=0; LCD_DrawSquareBox(left_x+1,LDtop_y+i*27,righ_x-2,24,Black);	// ???? clear_i_line?
				return;
			}
			ldflag=1;
			if(ldcxfg>hlg)return;else hlg-=ldcxfg;
			pos=LDtop_y;	color=Black;
lpLD2:
			//ÐÂÔö(½â¾öË¢ÐÂÓÐ²ÐÁô)
			ClearXY(pos,nob_x,26,670,Black);
			Dispsz16(hlg+1,4,pos,nob_x,color,White);								//nob
			Disprom(H_alm,0,Disp_ld[hlg][0],pos,alm_x,color,White); //alm      
			if(Disp_ld[hlg][1]>0&&Disp_ld[hlg][1]<=18)		//cjy
			{	Dispsz24(Disp_ld[hlg][1],2,pos,adr_x,color,White);		//adr_LL
				Dispsz24(Disp_ld[hlg][2],3,pos,adr_x+34,color,White);	//adr_AAA
			}
//			if(Disp_ld[hlg][1]>16&&Disp_ld[hlg][1]<=17)
//			{	i=(Disp_ld[hlg][1]-16)/32+1;
//				Dispsz24('L',1,pos,adr_x,color,White);
//				Dispsz24(i,1,pos,adr_x+12,color,White);									//ZQ_LL
//				Dispsz24(Disp_ld[hlg][2]%32,3,pos,adr_x+34,color,White);
//			}
//			i=xz[Disp_ld[hlg][1]][Disp_ld[hlg][2]];
//			Dispsz16(i,3,pos,val_x+18,color,White);		//adr_VAL
			time[4]=Disp_ld[hlg][4];
			time[3]=(Disp_ld[hlg][5]>>5)*8+(Disp_ld[hlg][6]>>5);
			time[2]=Disp_ld[hlg][5]&0x1f;time[1]=Disp_ld[hlg][6]&0x1f;
			time[0]=Disp_ld[hlg][7];
			Disptime(time+4,4,pos+4,tim_x,color,White);									//time
if(area!=0) {Dispsz16(area,2,pos+2,adr_x-18,color,White); } //area=0;
			waddr=0xa0000+(Disp_ld[hlg][1]-1)*6400+(Disp_ld[hlg][2]-1)*32;    
			SPI_Flash_Read(buf,waddr,3); 
			nam=buf[0];flo=buf[1];zon=buf[2];
if(nam>0x80) nam=nam-0x40;  //LQL
//USART1_SendByte(nam);	Delay16s(16,2);
			Disprom(H_nam,0,nam,pos,nam_x,color,White);									//dev_name
			if(flo>128)			  //Â¥²ã
			{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
				Dispsz24(flo,1,pos,flo_x+12,color,White);
			}
			else  { Dispsz24(flo,2,pos,flo_x,color,White); }
			Disprom(H_rom,Disp_ld[hlg][1],Disp_ld[hlg][2],pos,rom_x,color,White);//·¿¼äÃû³Æ	
			Disprom(H_zon,0,zon,pos,zon_x,color,White); 		//zon ·ÖÇø
			if(hlg>0)
			{	if(hlg==0)pos=LDtop_y;
				else {hlg--;pos+=27; if(pos<LDtop_y+5*27)goto lpLD2;}
			}			
		} //if(alm>1&&alm<8) end
		/*ÐÂÔö-¹ÊÕÏ²»ÏÔÊ¾*/
		if(alm>=16&&alm<=21&&BZ_CHECK==0) //¹ÊÕÏ ¹ÊÕÏ»Ö¸´
		{	
			if(gzzz==0)
			{	gzflag=0;
				i=0; LCD_DrawSquareBox(left_x+1,GZtop_y+i*27,righ_x-2,24,Black);	// ¹ÊÕÏ»Ö¸´ clear_i_line?
				return;
			}
			gzflag=1;
			if(gzcxfg>hlg) return; else hlg-=gzcxfg;	      
			pos=GZtop_y; color=Black;	//alm_x=24;	
lpGZ2:
//USART1_SendByte(pos/27);	Delay16s(16,2);
			//ÐÂÔö(½â¾öË¢ÐÂÓÐ²ÐÁô)
			ClearXY(pos,nob_x,26,670,Black);
			Dispsz16(hlg+1,4,pos,nob_x,color,White);		//nob
			if(alm>=16&&alm<=21)
			{	
				Disprom(H_alm,0,Disp_gz[hlg][0],pos,alm_x,color,White);//¹ÊÕÏÀàÐÍ
			}
			if(alm==21)
			{	i=gzzz;		//clear gzzz+1_line cjy
				if(i<=4)	//NO_Clear PingBi
				{LCD_DrawSquareBox(left_x+1,GZtop_y+i*27,righ_x-2,24,Black);} //¹ÊÕÏ»Ö¸´ clear_i_line?
			}	
			if(Disp_gz[hlg][2]==0||Disp_gz[hlg][1]>=129)	//sys_GZ
			{	if(Disp_gz[hlg][1]<=16&&Disp_gz[hlg][1]>0)
				{	Disprom(H_alm,0,65,pos,alm_x,color,White);
					Dispsz24(Disp_gz[hlg][1],4,pos,alm_x,color,White);
				}																										//»ØÂ·(¹ÊÕÏ/¶ÌÂ·)
//USART1_SendByte(Disp_gz[hlg][1]);	Delay16s(16,2);
				if(Disp_gz[hlg][1]==129)Disprom(H_alm,0,66,pos,alm_x,color,White);//Ö÷µç
				if(Disp_gz[hlg][1]==130)Disprom(H_alm,0,67,pos,alm_x,color,White);//±¸µç
				if(Disp_gz[hlg][1]==131)Disprom(H_alm,0,68,pos,alm_x,color,White);//Éù¹â
				if(Disp_gz[hlg][1]==133)Disprom(H_alm,0,70,pos,alm_x,color,White);//CRT¹ÊÕÏ
				if(Disp_gz[hlg][1]==135)
				{	Disprom(H_alm,0,18,pos,alm_x,color,White);
					Dispsz24(Disp_gz[hlg][2],2,pos,adr_x+34,color,White);	//AA
				}
				goto lpGZ;
			}
			else if(Disp_gz[hlg][1]>0&&Disp_gz[hlg][1]<=18)
			{	Dispsz24(Disp_gz[hlg][1],2,pos,adr_x,color,White);		//LL
				Dispsz24(Disp_gz[hlg][2],3,pos,adr_x+34,color,White);	//AA
			}
//			if(Disp_gz[hlg][1]>16&&Disp_gz[hlg][1]<=17)	//ZhiQi
//			{	
//				i=(Disp_gz[hlg][1]-16)+1;
//				Dispsz24('L',1,pos,adr_x,color,White);
//				Dispsz24(i,1,pos,adr_x+12,color,White);									//ZQ_LL
//				Dispsz24(Disp_gz[hlg][2]%32,3,pos,adr_x+34,color,White);
//			}
			waddr=0xa0000+(Disp_gz[hlg][1]-1)*6400+(Disp_gz[hlg][2]-1)*32;
			SPI_Flash_Read(buf,waddr,3);
			nam=buf[0];flo=buf[1];zon=buf[2];
if(nam>0x80) nam=nam-0x40;  //LQL
//USART1_SendByte(nam);	Delay16s(16,2);
			Disprom(H_nam,0,nam,pos,nam_x,color,White);
			if(flo>128)
			{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
				Dispsz24(flo,1,pos,flo_x,color,White);
			}
			else  Dispsz24(flo,2,pos,flo_x,color,White);
lpGZ:	time[4]=Disp_gz[hlg][4];
			time[3]=(Disp_gz[hlg][5]>>5)*8+(Disp_gz[hlg][6]>>5);
			time[2]=Disp_gz[hlg][5]&0x1f;time[1]=Disp_gz[hlg][6]&0x1f;
			time[0]=Disp_gz[hlg][7];
			Disptime(time+4,4,pos+4,tim_x,color,White);						 
if(area!=0) {Dispsz16(area,2,pos+2,adr_x-18,color,White); } //area=0;
			Disprom(H_rom,Disp_gz[hlg][1],Disp_gz[hlg][2],pos,rom_x,color,White);//ÇøÓò·¿¼äÃû³Æ
			Disprom(H_zon,0,zon,pos,zon_x,color,White); 		//zon ·ÖÇø
			//USART1_SendByte(hlg);	Delay16s(16,2);	
			if(hlg>0)
			{	
				if(hlg==0) pos=GZtop_y;	
				else {hlg--;pos+=27;	if(pos<GZtop_y+5*24)goto lpGZ2;}
			}			
		}  //if(alm>=16&&alm<=21) end
		if(alm==32)
		{	pos=17*24;	color=Black; //alm_x=24;
			LCD_DrawSquareBox(nob_x,pos,664 ,24,color);       //clear line
			if(xspbs==0)return;//if(pbcxfg>hlg)return;else hlg-=pbcxfg;
			Disprom(H_alm,0,Disp_pb[hlg][0],pos,alm_x,color,White);	//
			Dispsz24(Disp_pb[hlg][1],2,pos,adr_x,color,White);			//adr_LL
			Dispsz24(Disp_pb[hlg][2],3,pos,adr_x+34,color,White);		//adr_AAA
//			i=xz[Disp_pb[hlg][1]][Disp_pb[hlg][2]];
//			Dispsz16(i,3,pos,val_x+18,color,White);		//adr_VAL
			time[4]=Disp_pb[hlg][4];
			time[3]=(Disp_pb[hlg][5]>>5)*8+(Disp_pb[hlg][6]>>5);
			time[2]=Disp_pb[hlg][5]&0x1f;time[1]=Disp_pb[hlg][6]&0x1f;
			time[0]=Disp_pb[hlg][7];
			Disptime(time+4,5,pos+4,tim_x,color,White);
			Dispsz16(hlg+1,4,pos,nob_x,color,White);
			waddr=0xa0000+(Disp_pb[hlg][1]-1)*6400+(Disp_pb[hlg][2]-1)*32;
			SPI_Flash_Read(buf,waddr,3); 
			nam=buf[0];flo=buf[1];zon=buf[2];
if(nam>0x80) nam=nam-0x40;  //LQL
//USART1_SendByte(nam);	Delay16s(16,2);
			Disprom(H_nam,0,nam,pos,nam_x,color,White);									//dev_name
			if(zon<0xff)Dispsz24(zon,2,pos,zon_x,color,White);		//·ÖÇøÇøÓò
			else Dispsz24(0,2,pos,zon_x,color,White);
			if(flo<0xff)				          //Â¥²ã
			{	if(flo>128){flo=flo-128;Dispzf24('-',pos,flo_x,color,White);}
				else  Dispsz24(flo,2,pos,flo_x,color,White);
				Dispzf24('F',pos,flo_x,color,White);
			}
			Disprom(H_rom,Disp_pb[hlg][1],Disp_pb[hlg][2],pos,rom_x,color,White);//·¿¼äÃû³Æ 
		}

//		if(enter==1)Dispzf(0x1f,48,792,Red,Green);
//		if(enter==2)Dispzf(0x1f,192,792,Yellow,Green);
		return;
}	 
	 
void Logit(u8 alm,u8 area,u8 dll,u8 daa)
{u8 i,k,buf[15];
 u16 j;
 u32 waddr;
 //if(iwdgtim>200){iwdgtim=0;IWDG_ReloadCounter();}//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
//USART1_SendByte(area); Delay16s(16,2);USART1_SendByte(dll); Delay16s(16,2);
 switch(alm){
  case 0x0:alm=1;		//
  case 0x1:{Disp_hj[hjzz][0]=alm;
						Disp_hj[hjzz][1]=dll;
						Disp_hj[hjzz][2]=daa;
						Disp_hj[hjzz][3]=area;
		        Disp_hj[hjzz][4]=ctime[4];                         //Äê
	        	i=ctime[3]>>3;k=i<<5;Disp_hj[hjzz][5]=k+ctime[2];  //ÈÕ
			      i=ctime[3]&0x07;k=i<<5;Disp_hj[hjzz][6]=k+ctime[1];//Ê±
		        Disp_hj[hjzz][7]=ctime[0]; //·Ö
            Mdfjl(hjjltrue,0x80000);
			      if(hjjlzz<0x3e7)hjjlzz++;hjjltrue++;
			      if(hjjltrue==0x400)	//Ë¢ÐÂ¼ÍÂ¼
			          {hjjltrue=999; 
						     Mdfzz(0x80000);
				        }
						SPI_Flash_Read(buf,0x8c000,12);
	        	buf[0]=hjjlzz>>8;buf[1]=hjjlzz&0x00ff;
			      buf[2]=hjjltrue>>8;buf[3]=hjjltrue&0x00ff;
            SPI_Flash_Erase_Sector(140);
			      SPI_Flash_Write_Page(buf,0x8c000,12);
            if(hjzz==128){hjzz=0;return;}
            hjzz++;//Ë¢ÐÂÆÁÄ»»ð¾¯×ÜÊý
            break;}
   case 0x2://ÊÖÆô
   case 0x3:xslds++;//Æô¶¯,Ë¢ÐÂÆÁÄ»Æô¶¯×ÜÊý
   case 0x4://Í£Ö¹ 
   case 0x5://Î´´ð
   case 0x6://·´À¡
   case 0x7://·´À¡³·Ïû
	          Disp_ld[ldzz][0]=alm;Disp_ld[ldzz][1]=dll;Disp_ld[ldzz][2]=daa;Disp_ld[ldzz][3]=area;
						Disp_ld[ldzz][4]=ctime[4];                         //Äê
			      i=ctime[3]>>3;k=i<<5;Disp_ld[ldzz][5]=k+ctime[2];  //ÈÕ
						i=ctime[3]&0x07;k=i<<5;Disp_ld[ldzz][6]=k+ctime[1];//Ê±
						Disp_ld[ldzz][7]=ctime[0];                         //·Ö
            Mdfjl(ldjltrue,0x84000);  
            if(ldjlzz<0x3e7)ldjlzz++;ldjltrue++;
			      if(ldjltrue==0x400)	
			          {ldjltrue=999; 
						     Mdfzz(0x84000);
				        }		            
						SPI_Flash_Read(buf,0x8c000,12);
	        	buf[4]=ldjlzz>>8;buf[5]=ldjlzz&0x00ff;
			      buf[6]=ldjltrue>>8;buf[7]=ldjltrue&0x00ff;
            SPI_Flash_Erase_Sector(140);
			      SPI_Flash_Write_Page(buf,0x8c000,12);	
						if(alm==4){if(xslds!=0)xslds--;Modify(2,dll,daa);Modify(3,dll,daa);}
            if(alm==6){xshds++;Modify(5,dll,daa);}
            if(alm==7){if(xshds!=0)xshds--;if(xshds==0)LedFQ_off();Modify(6,dll,daa);}
					  if(ldzz==128){ldzz=0;return;}	
            ldzz++;//Ë¢ÐÂÆÁÄ»Áª¶¯×ÜÊý
            break;    						
   case 0x10://¹ÊÕÏ
   case 0x11://Ç·Ñ¹
   case 0x12:if(gzzz==255)gzzz=0;//¶ÌÂ·
   case 0x15://¹ÊÕÏ»Ö¸´
				if(alm==21)
				{	if(dll==0&&daa>0&&daa<=6) daa=daa;//LEDbuffer[daa-1]&=0xef; ZhiQi_addr
				}
				Disp_gz[gzzz][0]=alm;Disp_gz[gzzz][1]=dll;Disp_gz[gzzz][2]=daa;Disp_gz[gzzz][3]=area;
				Disp_gz[gzzz][4]=ctime[4];                         //Äê
				i=ctime[3]>>3;k=i<<5;Disp_gz[gzzz][5]=k+ctime[2];  //ÈÕ
				i=ctime[3]&0x07;k=i<<5;Disp_gz[gzzz][6]=k+ctime[1];//Ê±
				Disp_gz[gzzz][7]=ctime[0];                         //·Ö
				Mdfjl(tjltrue,0x88000);  
				if(tjlzz<0x3e7)	tjlzz++;	tjltrue++;
				if(tjltrue==0x400)	//Ë¢ÐÂ¼ÍÂ¼
				{	tjltrue=999; 
					Mdfzz(0x88000);
				}		         
				SPI_Flash_Read(buf,0x8c000,12);
				buf[8]=tjlzz>>8;buf[9]=tjlzz&0x00ff;
				buf[10]=tjltrue>>8;buf[11]=tjltrue&0x00ff;
				SPI_Flash_Erase_Sector(140);
				SPI_Flash_Write_Page(buf,0x8c000,12);	
				if(alm<0x15)gzzz++;
				if(alm==0x15)Modify(16,dll,daa);
//USART1_SendByte(gzzz+0x80);	Delay16s(16,2);	
//USART1_SendByte(Disp_gz[gzzz][0]);	Delay16s(16,2);						
//USART1_SendByte(Disp_gz[gzzz][2]);	Delay16s(16,2);		
				break;
   case 0x20:Disp_pb[xspbs][0]=alm;Disp_pb[xspbs][1]=dll;Disp_pb[xspbs][2]=daa;Disp_pb[xspbs][3]=area;
						 Disp_pb[xspbs][4]=ctime[4];                         //Äê
			       i=ctime[3]>>3;k=i<<5;Disp_pb[xspbs][5]=k+ctime[2];  //ÈÕ
						 i=ctime[3]&0x07;k=i<<5;Disp_pb[xspbs][6]=k+ctime[1];//Ê±
						 Disp_pb[xspbs][7]=ctime[0];                         //·Ö						
			       xspbs++;//Ë¢ÐÂÆÁ±Î×ÜÊý
   case 0x21:if(alm==0x21)Modify(32,dll,daa);
						 //ÐÞ¸ÄÆÁ±Î±í	
						 for(j=0;j<4096;j++)downb[j]=0xff;
             for(i=0;i<xspbs;i++){for(k=0;k<8;k++)downb[i*8+k]=Disp_pb[i][k];}
             downb[4095]=xspbs;
  			     SPI_Flash_Erase_Sector(143);
             SPI_Flash_Write_NoCheck(downb,0x8f000,4096);           	
			       SPI_Flash_Read(downb,0x8d000,4096); 
						 j=(dll-1)*256+daa;
             if(alm==0x20) downb[j]|=0x40;else downb[j]&=0xbf;		
  			     SPI_Flash_Erase_Sector(141);
             SPI_Flash_Write_NoCheck(downb,0x8d000,4096);           
             if(xspbs==0) {LedPB_off();}
						 else {LedPB_on();}
             goto loop;					 
   case 0x22: //¸´Î»
   case 0x23: //µÇ¼Ç
   case 0x24: //¿ª»ú
             if(alm==0x24)
						     {Get_time();  
									BKP_Write(); 
								  date=ctime[2];/////////////µ÷ÊÔ³ÌÐò
						     }
   case 0x25: //¹Ø»ú
             if(alm==0x25)BKP_Read();  
   case 0x26: //Áª¶¯¿ØÖÆ·½Ê½¸Ä±ä
 	 case 0x27: //Áª¶¯Æô¶¯
        loop:buf[0]=alm;buf[1]=dll;buf[2]=daa;buf[3]=area;
						 buf[4]=ctime[4];                          //Äê
			       i=ctime[3]>>3;k=i<<5;buf[5]=k+ctime[2];   //ÈÕ
						 i=ctime[3]&0x07;k=i<<5;buf[6]=k+ctime[1]; //Ê±
						 buf[7]=ctime[0];                          //·Ö 
			       waddr=0x88000+tjltrue*8;            //Ð´Èë¼ÇÂ¼
			       SPI_Flash_Write_Page(buf,waddr,8);
	           if(tjlzz<0x3e7)tjlzz++;tjltrue++;
			       if(tjltrue==0x400)	//Ë¢ÐÂ¼ÍÂ¼
			          {tjltrue=999; 
						     Mdfzz(0x88000);
				        }		  
							SPI_Flash_Read(buf,0x8c000,12);
//	buf[21]?			if(alm==0x26) buf[21]=manual_ld;   
	        	  buf[8]=tjlzz>>8;buf[9]=tjlzz&0x00ff;
			        buf[10]=tjltrue>>8;buf[11]=tjltrue&0x00ff;
              SPI_Flash_Erase_Sector(140);
			        SPI_Flash_Write_Page(buf,0x8c000,12);	
              if(printer&&(alm>0x21||alm<=7))Prntjl(alm,area,dll,daa);
              break;
   default:break;}
 return;
}



void Modify(u8 hf_alm,u8 dll,u8 daa)
{u8 i,k;u16 chg;
if(hf_alm>=16&&hf_alm<32)
 {for(i=0;i<8;i++) Disp_gz[gzzz][i]=0;
  if(daa!=0)  //Ì½²âÆ÷»Ö¸´
    {for(chg=0;chg<gzzz;chg++) //ÐÞ¸ÄÊý×éDisp_gz[][]
       {if((Disp_gz[chg][0]==hf_alm)&&(Disp_gz[chg][1]==dll)&&(Disp_gz[chg][2]==daa))
          {for(i=0;i<(gzzz-chg);i++){for(k=0;k<8;k++) Disp_gz[i+chg][k]=Disp_gz[i+chg+1][k];}
           gzzz--;}
       }
    }
  if(daa==0)  //»ØÂ·»Ö¸´
    {for(chg=0;chg<gzzz;chg++) //ÐÞ¸ÄÊý×éDisp_gz[][]
       {if((Disp_gz[chg][1]==dll)&&(Disp_gz[chg][2]==daa))
          {for(i=0;i<(gzzz-chg);i++){for(k=0;k<8;k++) Disp_gz[i+chg][k]=Disp_gz[i+chg+1][k];}
           gzzz--;}
       }
    }
  if(gzzz==0){if(hjflag==0)ZQbuffer&=0xf8;LedGZ_off();}//¹ÊÕÏÈ«²¿»Ö¸´
}
if(hf_alm==32)             //ÐÞ¸ÄÆÁ±Î±í
   {for(chg=0;chg<xspbs;chg++)   //ÐÞ¸ÄÊý×éDisp_pb[][]
       {if((Disp_pb[chg][0]==hf_alm)&&(Disp_pb[chg][1]==dll)&&(Disp_pb[chg][2]==daa))
           {for(i=0;i<(xspbs-chg);i++){for(k=0;k<8;k++) Disp_pb[i+chg][k]=Disp_pb[i+chg+1][k];}
            xspbs--;}
       }
    for(i=0;i<8;i++) Disp_pb[xspbs][i]=0;
   }
	 
if(hf_alm>1&&hf_alm<=9)
 {
  for(chg=0;chg<ldzz;chg++)       //ÐÞ¸ÄÊý×éDisp_ld[][]
     {if((Disp_ld[chg][0]==hf_alm)&&(Disp_ld[chg][1]==dll)&&(Disp_ld[chg][2]==daa))
          {for(i=0;i<(ldzz-chg);i++){for(k=0;k<8;k++) Disp_ld[i+chg][k]=Disp_ld[i+chg+1][k];}
           ldzz--;
          }
     }
  for(i=0;i<8;i++) Disp_ld[ldzz+1][i]=0;
 }	 
return;
}


void Mdfjl(u16 zz,u32 waddr)
{u8 i,buf[8];
 u32 addr;
 addr=waddr+zz*8;
 for(i=0;i<8;i++)buf[i]=0;
 if(waddr==0x80000){for(i=0;i<8;i++) buf[i]=Disp_hj[hjzz][i];}
 if(waddr==0x84000){for(i=0;i<8;i++) buf[i]=Disp_ld[ldzz][i];}//Áª¶¯¼ÇÂ¼
 if(waddr==0x88000){for(i=0;i<8;i++) buf[i]=Disp_gz[gzzz][i];}
 SPI_Flash_Write_Page(buf,addr,8);
 return;
}


void Mdfzz(u32 waddr)
{u16 i;u8 j;	
 if(waddr==0x80000)j=128;//»ð¾¯¼ÇÂ¼
 if(waddr==0x84000)j=132;//Áª¶¯¼ÇÂ¼
 if(waddr==0x88000)j=136;//¹ÊÕÏ¼ÇÂ¼
 SPI_Flash_Read(downb,waddr,8192);										
 for(i=1024;i<8192;i++)downb[i-1024]=downb[i];//128Ìõ*8×Ö½Ú
 SPI_Flash_Erase_Sector(j);SPI_Flash_Erase_Sector(j+1);
 SPI_Flash_Write_NoCheck(downb,waddr,7168);			
 for(i=0;i<8192;i++) downb[i]=0; 	 
return;
}
 


void liandongBC(uchar dll,uchar daa)
{	u16 pf,pn,k,num;													//Âú×ãÌõ¼þ cbk[n]=0x80
	u8 j,cond=0,sum,ts,ld=0,hx=0,lg,x,y,buf[90]; //i,ll,aa,
	u32 waddr,daddr=0,temp=0,dloop;
	SPI_Flash_Read(downb,0xef000,2048); 
	num=downb[2046]<<8;num=num+downb[2047];if(num>250)return;//ÎÞ±à³Ì
//USART1_SendByte(num>>8);Delay16s(16,2);	USART1_SendByte(num&0xff);Delay16s(16,2);	
	dloop=dll*1000+daa;
	for(k=0;k<num;k++)
	{	////if(cxb[k]==0xff)continue;		//ÒÑÖ´ÐÐ¹ý£¬±»Rfsh Çå³É 0xff But_QD_TZ_Dont_ReQD
		sum=0;ts=0;
		pf=downb[k*2]<<8;pf=pf+downb[k*2+1];pn=downb[k*2+2]<<8;pn=pn+downb[k*2+3];
		lg=pn-pf;	     																	//pf  pn
		waddr=0xe0000+pf;SPI_Flash_Read(buf,waddr,lg);
		cond=buf[0];
		for(j=1;j<80;)						   
//loop2:
		{	x=buf[j];j++;
			if(x==0xfb)break;//Ìõ¼þ½áÊø
			y=buf[j];j++;
		                if((cond==0x22)&&(x==0)&&(y==0)){ts=1;continue;}
			if(x>0x80)
			{	temp=(x-0x80)*1000+y;hx=1;continue;}
			else
			{	daddr=1000*x+y; }
			if(hx==1)
			{	if(dloop>=temp&&dloop<=daddr)
				{	if(cond==0x22)
					{	if(ts==0)	cxb[k]|=0x01;		//Âú×ãÌõ¼þ3µÚÒ»¶Î
						else 			cxb[k]|=0x10;}  //Âú×ãÌõ¼þ3µÚ¶þ¶Î
						ld=1;
				}
				sum+=daddr-temp+1;hx=0;continue;
			}
			if(daddr==dloop)
			{	if(cond==0x22)
				{	if(ts==0)	cxb[k]|=0x01;			//Âú×ãÌõ¼þ3µÚÒ»¶Î
					else 			cxb[k]|=0x10;
				}															//Âú×ãÌõ¼þ3µÚ¶þ¶Î
				ld=1;
			}
			sum++;
		}
		if(ld)
		{	ld=0;	       
			switch(cond)
			{						//Ìõ¼þ
				case 0x26://Óë
					cxb[k]++;	if(cxb[k]==sum) goto loop1;
					break;
				case 0x2b://»ò
					loop1:	cxb[k]=0x80;ldqdfg++;
//USART1_SendByte(k);Delay16s(16,2);	USART1_SendByte(cxb[k]);Delay16s(16,2);	
					break;
				case 0x2a://2µã»ò
					cxb[k]++;	if(cxb[k]>=2) goto loop1;
					break;
				case 0x22://2¶ÎÓë
					if((cxb[k]&0x11)==0x11)goto loop1;
					break;
				default:	break;
			}
		}
	}
}

void liandongQD(u16 p,u8 fa)
{	u16 pf,pn,daddr=0,temp;  //dloop,
	u8 i,j,hx=0,ll,aa,lg,x,y,buf[200]; //,by=0x1
	u32 waddr;
	waddr=0xef000+p*2;SPI_Flash_Read(buf,waddr,4); //Ö¸Õë±í ¶ÔÓ¦¶Î
	pf=buf[0]<<8;pf=pf+buf[1];pn=buf[2]<<8;pn=pn+buf[3]; //Ö¸Õë±í µ±Ç°¶Î-ºó¶Î
	lg=pn-pf;
	waddr=0xe0000+pf;SPI_Flash_Read(buf,waddr,lg);	  //Áª¶¯ µ±Ç°¶Î  
	for(i=0;i<lg;i++)	{	if(buf[i]==0xfb)break;}       //Ìø¹ý Áª¶¯¹ØÏµ
	for(j=i+1;;j=j+2)						   
	{	x=buf[j];																				//±»ÆôµØÖ· daddr
		if(x!=0xfc)
		{	x=buf[j];y=buf[j+1];
			if(x>0x80)
			{	temp=(x-0x80)*1000+y;hx=1;goto loop;}
			else {	daddr=1000*x+y;	}	 
			if(hx==1)															//from to Á¬ÐøÆô¶¯
			{	do
				{	ll=daddr/1000;aa=daddr%1000;
//USART1_SendByte(ll);Delay16s(16,2);	USART1_SendByte(aa);				
					if((ll>0)&&(ll<=16))
					{
						if((djb[ll-1][aa]&0xed)==0x81)
						{	djb[ll-1][aa]|=0x20; //VZQD++;
							HJalarm(3,0,ll,aa);
							lpxq_bz[ll-1]=0x02;
							U7_QDHDcmd(3,(ll-1)*256+aa);
//							CRT_alarm(3,ll,aa,0);                //QD_CRT
//USART1_SendByte(aa);
						}
					}
					if(ll==17)						//×¨ÏßÁª¶¯
					{
						if((djb[ll-1][aa]&0xa1)==0x81)
						{
//							{	djb[ll-1][aa] |=0x20; 
//							lpxq_bz[ll-1]=0x02; //VZQD++;
//							HJalarm(3,0,ll,aa);
							if((aa&0x01)==1)	{ U7_txQD[2][(aa-1)/2] |=0x40; }
							else      				{ U7_txQD[2][(aa-1)/2] |=0x04; }
//USART1_SendByte(aa);
						}							
//								CRT_alarm(3,ll,aa,0);                //QD_CRT
//USART1_SendByte(0xb3);
//							}
					}
					daddr--;
				}while(daddr>=temp);
				hx=0;goto loop;
			}
			ll=daddr/1000;aa=daddr%1000;
//USART1_SendByte(ll);	Delay16s(16,2);	USART1_SendByte(aa);	Delay16s(16,2);					
			if((ll>0)&&(ll<=16))
			{	
//USART1_SendByte(ll);	Delay16s(16,2);	USART1_SendByte(aa);	Delay16s(16,2);					
				if((djb[ll-1][aa]&0xed)==0x81)
				{	djb[ll-1][aa]|=0x20;  //VZQD++;
					HJalarm(3,0,ll,aa);
					lpxq_bz[ll-1]=0x02;
					U7_QDHDcmd(3,(ll-1)*256+aa);
//					CRT_alarm(3,ll,aa,0);                //QD_CRT
//USART1_SendByte(ll);Delay16s(16,2);	USART1_SendByte(aa);				
				}
			}				//µÇ,Î´Æô,ÎÞÆÁ¹Ê
//USART1_SendByte(aa);					
			if(ll==17)	
			{	//×¨ÏßÁª¶¯
//USART1_SendByte(ll);	Delay16s(16,2);	USART1_SendByte(aa);	Delay16s(16,2);					
//				if((djb[ll-1][aa]&0xa1)==0x81)
				{	//djb[ll-1][aa]|=0x20; //VZQD++;
//					HJalarm(3,0,ll,aa);
//					CRT_alarm(3,ll,aa,0);                //QD_CRT
					if((aa&0x01)==1)	{ U7_txQD[2][(aa-1)/2] |=0x40; }
					else      				{ U7_txQD[2][(aa-1)/2] |=0x04; }
//USART1_SendByte(aa);					
				}					
			}
			goto loop;
		}
		loop:if(x==0xfc)goto loop1;         //½áÊø  
	}
	loop1:return;
}

void	LianDongQR(void)
{
	USART1_SendByte(0xba);
}

void ZiJian(void)
{	u8 i,buf[3];
  for(i=0;i<3;i++) buf[i]=LEDbuffer[i]; //save LED statu
//LDGB_zijian 0x6e(µÚ2´®¿Ú) 
	LDGBstrb[2]=0; LDGBstrb[3]=0x6e;LDGBstrb[4]=0;

	U7_cmdbz[2] =0x1a;				//0x12  |=0x08
	Usart_loop7ZhiQi(2);			//ZhiQipan   zhi_jian
	U2_cmdbz[0]=0x07;
	Usart_loop2ZXP(0);				//zongxianp   zhi_jian
	U2_cmdbz[0]=0x02;					//ZXP zijian end
/*	Ur485_1=1;
	for(i=0;i<12;i++)			// from 1 to LEN
	{	j=LDGBstrb[i];			//LDGB
		USART2->DR = (j & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
	}
	Delay16s(60,200); //400us
	Ur485_1=0;
*/
//	LEDbuffer[0]=0xff;
	LEDbuffer[1]=0xff;LEDbuffer[2]=0xff;
	KYXHJ_on();KZYX_on();
	ClearS(0,0,800,480,Red);//ºì
//	Disphzs24(0xd7d4,200,376,Red,White);Disphzs24(0xbcec,200,400,Red,White);
	Delay1ms(2000);	KLCD_off();	Delay1ms(2000);	KLCD_on();
	KYXLD_on();
	ClearS(0,0,800,480,Blue);//À¶
	Delay1ms(2000);KLCD_off();	Delay1ms(2000);KLCD_on();
	KYXGZ_on();
	ClearS(0,0,800,480,Yellow);//»Æ
	Delay1ms(2000);KLCD_off();	Delay1ms(2000);KLCD_on();
	KZYX_off();
//	LEDbuffer[0]=0x00;
	U7_cmdbz[2] =0x12;				//0x12  |=0x08
	Usart_loop7ZhiQi(2);			//ZhiQipan   zhi_jian
	LEDbuffer[1]=0x00;LEDbuffer[2]=0x00;
  for(i=0;i<3;i++)LEDbuffer[i]=buf[i]; //restore LED
  Esc();
 	fk8save=0;	fk4save=3;	DispMenu(0,0);							//ÏÔÊ¾²Ëµ¥
	DispMenu(1,0);	DispMenu(2,0);	DispMenu(3,0);	DispMenu(4,0);
}	
//void ZiJian(void)
//  {u8 i,buf[3];
//   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
//   for(i=0;i<3;i++)buf[i]=LEDbuffer[i]; 
//   LEDbuffer[0]=0x0f;LEDbuffer[1]=LEDbuffer[2]=0xff;
//	 ClearS(0,0,800,480,Red);//ºì
//	 Disphzs24(0xd7d4,200,376,Red,White);Disphzs24(0xbcec,200,400,Red,White);
//   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
//	 ZQbuffer&=0xf8;ZQbuffer|=0x08;
//   ZQbuffer&=0xf8;ZQbuffer|=0x03;//Ïû·À³µÉù
//	 GPIOI->ODR=ZQbuffer; 
//   ZQZJ_on();Delay16s(20,20); 
//   Ledscan_off();
//   Delay16s(20000,8000);
//   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
//	 ClearS(0,0,800,480,Green);//»Æ
//	 Disphzs24(0xd7d4,200,376,Green,Black);Disphzs24(0xbcec,200,400,Green,Black);
//	 ZQbuffer&=0xf8;ZQbuffer|=0x05;  //»ú¹ØÇ¹Éù
//	 GPIOI->ODR=ZQbuffer;
//	 ZQZJ_on();Delay16s(20,20);     
//   Ledscan_off();
//   Delay16s(20000,8000);
//   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
//	 ClearS(0,0,800,480,Blue);//À¶
//	 Disphzs24(0xd7d4,200,376,Blue,White);Disphzs24(0xbcec,200,400,Blue,White);
//	 ZQbuffer&=0xf8;ZQbuffer|=0x01;  //¾È»¤³µÉù
//	 GPIOI->ODR=ZQbuffer;  
//	 ZQZJ_on();Delay16s(20,20);
//   Ledscan_off();
//   Delay16s(20000,8000); 
//   ZQbuffer&=0xf8;//ÉùÍ£ 
//   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
//   for(i=0;i<3;i++)LEDbuffer[i]=0;
//   for(i=0;i<3;i++)LEDbuffer[i]=buf[i];
//   Esc();
//   }

void ResetC(void)
 {u8 i,j;
//	u8 txbuf[12]={0xf0,0x09,0x00,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0xf2,0x00};
	u16 k;
	Logit(34,0,0,0);    //Ð´Èë¼ÇÂ¼
  enter=0;Delay16ms=0;lphd=0;
  //LDGB¸´Î»0x6f(µÚ2´®¿Ú) 
//	LDGBstrb[2]=0; LDGBstrb[3]=0x6f;LDGBstrb[4]=0;
//	Ur485_1=1;
//	for(i=0;i<12;i++)			// from 1 to LEN
//	{	j=LDGBstrb[i];			//LDGB
//		USART2->DR = (j & (uint16_t)0x01FF);
//		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
//	}
//	Delay16s(60,200); //400us
//	Ur485_1=0;
//	ZQP_Cmd57=0;
	 KZSG_off();			//SG change HJ_JJ
		CRT_Fuwei();					//CRT¸´Î» ·¢ËÍÃüÁî
		for(i=0;i<=200;i++)
		{ CRT_txbuf[i][0] =0x0;}				//CRT¸´Î» ·¢ËÍÃüÁî
		for(k=1;k<512;k++)
		{		SGGB[k] &=0x7fff;	}
		bzSGGB =0;
		QuYu_Fuwei();					//quyu¸´Î» ·¢ËÍÃüÁî
	//	for(i=0;i<=31;i++)
//	{ ZQP_txbuf[i][0] =0x0;}					//ZQP¸´Î» ·¢ËÍÃüÁî
		for(j=0;j<32;j++) {	U7_txQD[2][j] &=0xbb;	djb[16][j+1] &=0x81;	} //¸´Î»Zhiqi bit
		U7_cmdbz[2] =0x16;				//0x12  |=0x04
		Usart_loop7ZhiQi(2);			//ZhiQipan¸´Î»
		for(j=0;j<32;j++) {	U7_rxKB[2][j] &=0xbb;		} //¸´Î»Zhiqi bit
	Delay1ms(1000);  //1s  
		U7_cmdbz[2] =0x16;				//0x12  |=0x04
		Usart_loop7ZhiQi(2);			//ZhiQipan¸´Î»
	ClearS(0,0,800,480,Blue);	//kill re_Disp ZhiQi
		U2_cmdbz[0]=0x06;
		Usart_loop2ZXP(0);				//zongxianp   Fuwei
		U2_cmdbz[0]=0x02;					//ZXP Fuwei end
		for(i=0;i<4;i++) for(j=0;j<16;j++)
		{ U2_txQD[i][j] =0x0; U2_txHD[i][j] =0x0; }					
//	for(j=0;j<16;j++) {	U7_txQD[2][j] &=0xbb;	djb[16][j] &=0x81;	} //¸´Î»Zhiqi bit


// CengX ¸´Î»  F0 05 c9 05 F2(µÚÒ»´®¿Ú) 
		for(i=0;i<16;i++)
	{	GPIOI->ODR=i;
		Lpscan_on();Delay16s(16,1);
		Ledscan_off();

		USART3->DR = 0xf0;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0x05;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0xc9;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
		USART3->DR = 0x05;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0xf2;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		Delay1ms(10);
	}
	do{}while(lpfresh<20);lpfresh=0;//1sÄÚ»ØÂ·Êý¾ÝÎÞÐ§ 

//»ØÂ·¸´Î»F0 0x6 F2(µÚÒ»´®¿Ú) 
	for(i=0;i<16;i++) lpxq_bz[i]=1;  //sub basic out cmd
		
	for(i=0;i<16;i++)
	{	GPIOI->ODR=i;
		Lpscan_on();Delay16s(16,1);
		Ledscan_off();

		USART3->DR = 0xf0;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0x05;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0xc9;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
		USART3->DR = 0x05;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0xf2;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		Delay1ms(10);

		USART3->DR = 0xf0;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0x06;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);	 
		USART3->DR = 0xf2;
		while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
		Delay1ms(10);
	}
	for(i=0;i<16;i++) for(j=0;j<201;j++)
	{	lpxz_val[i][j]=0; lpxz_bit[i][j]=0; QYdjb[i][j]=0; }

	do{}while(lpfresh<20);lpfresh=0;//1sÄÚ»ØÂ·Êý¾ÝÎÞÐ§
		
	lpflag_hj=0;lpflag_gz=0;lpflag_ad=0;lpsection=0;lpzz=0;
	inpzz3=0;prlong3=0;inpzz=0;prlong=0;
	//Çå¼Ä´æÆ÷
	moniBZ=0;	
	VZHJ=0;	VZQD=0;VZHD=0;VZGZ=0;VZSX=0;  VZQDHD=0;VZZQ=0; 
  key=0;ROWfirst=ROWsecond=0;
  MenuC=MenuF=MenuN=0;Top=0;cifg=0;xhzz=0;opfg=0;
  zero=0;jlend=0;djbloop=1;djbaddr=1;  djbcount=0;
  hjjlfg=qtjlfg=ldjlfg=djfg=glfg=djbfg=gzbfg=pbbfg=0;xslds=xshds=0;qdhdfg=0;
  hjcxfg=gzcxfg=ldcxfg=0;enter=0;zdgz=bdgz=sggz=0;
  for(i=0;i<12;i++)shu[i]=0;knum=0;password=0;
	for(i=0;i<16;i++){for(j=0;j<201;j++){xz[i][j]=0;hjjs[i][j]=0;gzjs[i][j]=0;unasw[i][j]=0;}}
  for(i=0;i<16;i++){for(j=1;j<201;j++)djb[i][j]&=0xd1;}
	for(i=0;i<16;i++){for(j=0;j<201;j++)CI[i][j]=0;}
  for(k=0;k<hjzz;k++){for(j=0;j<8;j++)Disp_hj[k][j]=0;}//Çåµ±Ç°ÊÂ¼þRAM
  for(k=0;k<gzzz;k++){for(j=0;j<8;j++)Disp_gz[k][j]=0;}
	for(k=0;k<ldzz;k++){for(j=0;j<8;j++)Disp_ld[k][j]=0;}
  for(k=0;k<500;k++)cxb[k]=0;ldqdfg=0;
	lpcxzz=0;lpcxfg=0;for(i=0;i<40;i++){for(j=0;j<10;j++)lpcxb[i][j]=0;}//Çå»ØÂ·²ãÏÔ±í
  for(i=0;i<8;i++){for(j=0;j<12;j++){pack2[i][j]=0;pack6[i][j]=0;pack7[i][j]=0;}}
  for(i=0;i<32;i++){for(j=0;j<12;j++){strbup[i][j]=0;strbuf[i][j]=0;}}
	for(i=80;i<100;i++){for(j=0;j<6;j++)strblp[i][j]=0;}	
	cicycle=1;				
  for(k=0;k<8128;k++)downb[k]=0;dwpack=0;
  wtsum=tsum=0;wtsumup=tsumup=0;trswait=trswaitup=0;inpzzhd=inpzzhd7=0;
  //outpzz1=inpzz1=psum1=prlong1=0;
	outpzz2=inpzz2=psum2=prlong2=0;XJ2=0;
	outpzz6=inpzz6=psum6=prlong6=0;XJ6=0;
	outpzz7=inpzz7=psum7=prlong7=0;
  hjzz=gzzz=ldzz=0;hjflag=gzflag=ldflag=0;Spalarm=0;
  LEDbuffer[1]=LEDbuffer[2]=0;ZQbuffer=0;//ÏûÒô 
//	for(j=0;j<16;j++) {	U7_txQD[2][j] &=0xbb;	djb[16][j] &=0x81;	} //¸´Î»Zhiqi bit
  //LEDbuffer[0]=0x1;            //Ö÷µçµÆÁÁ
	KZHJ_off();
  if(xspbs>0) LedPB_on();
	if(autofg==0){LedZD_off();LedSD_on();}
  else         {LedSD_off();LedZD_on();}
  if(CRTxj>0)RS485_down(0x7f,machine,0x6f,0,0,0,0,0);//¸´Î»CRT
	
	for(j=0;j<32;j++) {	U7_txQD[2][j] &=0xbb;	djb[16][j+1] &=0x81;	} //¸´Î»Zhiqi bit
	U7_cmdbz[2] =0x16;				//0x12  |=0x04
	Usart_loop7ZhiQi(2);			//ZhiQipan¸´Î»

	Delay1ms(1000);  //1s  
  Esc();
 	fk8save=0;	fk4save=3;	DispMenu(0,0);							//ÏÔÊ¾²Ëµ¥
	DispMenu(1,0);	DispMenu(2,0);	DispMenu(3,0);	DispMenu(4,0);
	KLCD_on();

  return;
}
 

void Down(void)
{	if(moniBZ==1)
	{ if(moniLL<=16) moniLL+=1;	}
	Tlcd=0;  //LCD_on	Top=0;
//if(hjjlfg!=0&&jlend==0)Displine(1,hjjlfg);
//if(qtjlfg!=0&&jlend==0)Displine(3,qtjlfg);
////if(cifg!=0){MenuN=23;ClearXY(430,595,24,205,White);Dispzf('_',430,640,White,Black);}//CIcx();
//if(ldfg!=0){MenuN=24;ClearXY(407,380,24,420,White);dstr=LDMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);}//LiandongQT();
//if(glfg!=0){MenuN=25;ClearXY(407,380,24,420,White);dstr=GLMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);}//Geli();
//if(djfg!=0){MenuN=14;ClearXY(407,380,24,420,White);dstr=DJMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);}//DengjiSD();
//if(djbfg!=0)
//   {if(djbaddr>200)
//       {if(djbloop<8)djbloop++;else djbloop=1;
//		    djbaddr=1;
//		   }
//    DJBcx(djbloop);
//	 }
//if(pbbfg!=0){if(xspbs>(12*pbbfg))PBBcx(pbbfg);}
//if(gzbfg!=0){if(gzzz>(12*gzbfg))GZcx(gzbfg);}
//if(enter==1){if(hjzz>(6+hjcxfg))Dispram(1,hjzz-1,1,0);}
//if(enter==3){if(gzzz>gzcxfg)Dispram(16,gzzz-1,3,0);}
//if(enter==2){if(ldzz>(5+ldcxfg))Dispram(2,ldzz-1,2,0);}
//Delay16s(3000,3000);
	if(	chaType==0)  //µ±Ç°
	{	if(dispZON==0x01)
		{	if(hjzz<(hjzzbak-1))
			{	hjzz+=1;	Dispram(1,hjzz,0,0);}
		}
		if(dispZON==0x02)
		{	if(ldzz<(ldzzbak-1))
			{	ldzz+=1;	Dispram(2,ldzz,0,0);}
		}
		if(dispZON==0x03)
		{	if(gzzz<(gzzzbak-1))
			{	gzzz+=1;	Dispram(16,gzzz,0,0);	}
		}
	}
	if(chaType==1)  //µµ°¸
	{	if((dispZON==1)&&(ChaHJpg>0))
		{	Displine_JLB(dispZON,ChaHJpg);
			ChaHJpg-=1;
		}
		if((dispZON==2)&&(ChaLDpg>0))
		{	Displine_JLB(dispZON,ChaLDpg);
			ChaLDpg-=1;
		}
		if((dispZON==3)&&(ChaGZpg>0))
		{	Displine_JLB(dispZON,ChaGZpg);
			ChaGZpg-=1;
		}
	}	
	if(chaType==3)    //µÇ¼Ç
	{	DJBcx(1);
	}
	/*ÐÂÔö*/
	if(chaType==4)		//¼ì²é
	{
		Checkpg+=1; 
		if(Checkpg>TypeCnt-9){
			Checkpg = TypeCnt-9;
		}
		DispCheck_JLB(Checkpg);
	}
	/*ÐÂÔö½áÊø*/

//	USART1_SendByte(55);
return;
}


void Up(void)
{	if(moniBZ==1)
	{ if(moniLL>1) moniLL-=1;	}
	Tlcd=0;	//Top=0;
//	if(hjjlfg!=0){hjjlfg=0;jlend=0;Displine(1,0);}
//	if(qtjlfg!=0){qtjlfg=0;jlend=0;Displine(3,0);}
//	if(djbfg!=0){djbfg=0;djbaddr=1;djbloop=1;DJBcx(djbloop);}
//	if(pbbfg!=0){pbbfg=0;PBBcx(0);}
//	if(gzbfg!=0){gzbfg=0;GZcx(0);}
//	if(enter==1){hjcxfg=0;Dispram(1,hjzz-1,0.0);}
//USART1_SendByte(dispZON);Delay16s(20,20);
	if(	chaType==0)  //µ±Ç°
	{	if(dispZON==0x01)
		{	if(hjzz>=5)
			{	hjzz-=1;Dispram(1,hjzz,0,0);//		gzcxfg=0;
			}
		}
		if(dispZON==0x02)
		{	if(ldzz>=5)
			{	ldzz-=1;Dispram(2,ldzz,0,0);//		gzcxfg=0;
			}
		}
		if(dispZON==0x03)
		{	if(gzzz>=5)
			{	gzzz-=1;Dispram(16,gzzz,0,0); //		gzcxfg=0;
			}
		}
	}
	if(chaType==1)    //µµ°¸
	{	if(dispZON==1)
		{	Displine_JLB(dispZON,ChaHJpg);
			ChaHJpg+=1;
		}
		if(dispZON==2)
		{	Displine_JLB(dispZON,ChaLDpg);
			ChaLDpg+=1;
		}
		if(dispZON==3)
		{	Displine_JLB(dispZON,ChaGZpg);
			ChaGZpg+=1;
		}
	}	
	if(chaType==3)    //µÇ¼Ç
	{	DJBcx(2);
	}
	/*ÐÂÔö*/
	if(chaType==4)		//¼ì²é
	{
		Checkpg-=1;
		if(Checkpg<0){
			Checkpg = 0;
		}
		DispCheck_JLB(Checkpg);
	}
	/*ÐÂÔö½áÊø*/
//	if(enter==2){ldcxfg=0;Dispram(2,ldzz-1,0,0);}
//	Delay16s(300,300);
//USART1_SendByte(56);
return;
}		 

void Left(void)
{Tlcd=0;	//Top=0;
//	if(hjjlfg!=0){hjjlfg=0;jlend=0;Displine(1,0);}
//	if(qtjlfg!=0){qtjlfg=0;jlend=0;Displine(3,0);}
//	if(djbfg!=0){djbfg=0;djbaddr=1;djbloop=1;DJBcx(djbloop);}
//	if(pbbfg!=0){pbbfg=0;PBBcx(0);}
//	if(gzbfg!=0){gzbfg=0;GZcx(0);}
//	if(enter==1){hjcxfg=0;Dispram(1,hjzz-1,0);}
//USART1_SendByte(dispZON);Delay16s(20,20);
//USART1_SendByte(57);
	if(dispZON==0x01)
	{	if(hjzz>=5)
		{//		gzcxfg=0;
			hjzz-=1;
			Dispram(1,hjzz,0,0);
		}
	}
	if(dispZON==0x02)
	{	if(ldzz>=5)
		{//		gzcxfg=0;
			ldzz-=1;
			Dispram(2,ldzz,0,0);
		}
	}
	if(dispZON==0x03)
	{	if(gzzz>=6)
		{//		gzcxfg=0;
			gzzz-=5;
			Dispram(16,gzzz,0,0);
		}
	}
	if(chaType==3)    //µÇ¼Ç
	{	DJBcx(3);      // -> loop+1
	}
//	if(enter==2){ldcxfg=0;Dispram(2,ldzz-1,0,0);}
//	Delay16s(300,300);
return;

}
void Right(void)
{Tlcd=0;//Top=0;
//	if(hjjlfg!=0){hjjlfg=0;jlend=0;Displine(1,0);}
//	if(qtjlfg!=0){qtjlfg=0;jlend=0;Displine(3,0);}
//	if(djbfg!=0){djbfg=0;djbaddr=1;djbloop=1;DJBcx(djbloop);}
//	if(pbbfg!=0){pbbfg=0;PBBcx(0);}
//	if(gzbfg!=0){gzbfg=0;GZcx(0);}
//	if(enter==1){hjcxfg=0;Dispram(1,hjzz-1,0,0);}
//USART1_SendByte(dispZON);Delay16s(20,20);
//USART1_SendByte(58);
	if(dispZON==0x01)
	{	if(hjzz<(hjzzbak-1))
		{//		gzcxfg=0;
			hjzz+=1;
			Dispram(1,hjzz,0,0);
		}
	}
	if(dispZON==0x02)
	{	if(ldzz<(ldzzbak-1))
		{//		gzcxfg=0;
			ldzz+=1;
			Dispram(2,ldzz,0,0);
		}
	}
	if(dispZON==0x03)
	{	if(gzzz<(gzzzbak-5))
		{//		gzcxfg=0;
			gzzz+=5;
			Dispram(16,gzzz,0,0);
		}
	}
	if(chaType==3)    //µÇ¼Ç
	{	DJBcx(4);      // <- loop-1
	}

return;

}
void ChaHJ(void)
{	if(dispZON==2) {ldzz=ldzzbak; dispZON=0;}
	if(dispZON==3) {gzzz=gzzzbak; dispZON=0;}
	if(dispZON==0)
	{	dispZON=1;
		Disphzs(0xb2e9,64+120*0,4,Blue,Magenta);
		LCD_DrawSquareBox(4,64+120*1,16,16,Blue);
		LCD_DrawSquareBox(4,64+120*2,16,16,Blue);
		hjzzbak=hjzz; 
	}
	else
	{	if(dispZON==1)
		{	dispZON=0;
			LCD_DrawSquareBox(4,64+120*0,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*1,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*2,16,16,Blue);
			hjzz=hjzzbak;
		}
	}
}
void ChaLD(void)
{	if(dispZON==1) {hjzz=hjzzbak; dispZON=0;}
	if(dispZON==3) {gzzz=gzzzbak; dispZON=0;}
	if(dispZON==0)
	{	dispZON=2;
		Disphzs(0xb2e9,64+120*1,4,Blue,Magenta);
		LCD_DrawSquareBox(4,64+120*0,16,16,Blue);
		LCD_DrawSquareBox(4,64+120*2,16,16,Blue);
		ldzzbak=ldzz;
	}
	else
	{	if(dispZON==2)
		{	dispZON=0;
			LCD_DrawSquareBox(4,64+120*0,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*1,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*2,16,16,Blue);
			ldzz=ldzzbak;
		}
	}
}
void ChaGZ(void)
{	if(dispZON==2) {ldzz=ldzzbak; dispZON=0;}
	if(dispZON==1) {hjzz=hjzzbak; dispZON=0;}
	if(dispZON==0)
	{	dispZON=3;
		Disphzs(0xb2e9,64+120*2,4,Blue,Magenta);
		LCD_DrawSquareBox(4,64+120*0,16,16,Blue);
		LCD_DrawSquareBox(4,64+120*1,16,16,Blue);
		gzzzbak=gzzz;
	}
	else
	{	if(dispZON==3)
		{	dispZON=0;
			LCD_DrawSquareBox(4,64+120*0,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*1,16,16,Blue);
			LCD_DrawSquareBox(4,64+120*2,16,16,Blue);
			gzzz=gzzzbak; 
//  		USART1_SendByte(0x47);
		}
	}
}

void Prntjl(u8 alm,u8 area,u8 dll,u8 daa)
{u8 i,j=0,x=0,buf[26],fwbuf[100],nam,flo,zon;
 u32 waddr[4];	
 for(j=0;j<20;j++) {if((j%6)==0)fwbuf[j]=0x2a;else fwbuf[j]=0x20;}
 fwbuf[j++]=0x0a;//´òÓ¡·Ö¸ô·û(5¸ö*),µ½±ß½ç×Ô¶¯»»ÐÐ
 fwbuf[j++]=0x1c;fwbuf[j++]=0x26;
 if(alm<=1) waddr[0]=H_alm+8;else waddr[0]=H_alm+alm*8;
 if(dll>128||(daa==0&&dll>0))
 {//ËÄ¸öºº×Ö
	if(dll<=8)x=65;//»ØÂ·
  if(dll==129)x=66;//Ö÷µç
  if(dll==130)x=67;//±¸µç
	if(dll==131)x=68;//Éù¹â
  if(dll==132)x=69;//Â¥ÏÔ
	if(dll==134)x=70;//CRT
	if(dll==151)x=81;
	if(dll==152)x=82;
  waddr[1]=H_alm+x*8;
	SPI_Flash_Read(buf,waddr[0],4);for(i=0;i<4;i++)fwbuf[j++]=buf[i];
  fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ
	SPI_Flash_Read(buf,waddr[1],12);
	for(i=0;i<4;i++)fwbuf[j++]=buf[i];//ÌØÊâÉè±¸2¸ö×Ö
  goto loop2;
 }
 if(dll==0&&daa==0)
 {//Á½¸öºº×Öfwbuf[j++]=0x0a;
	SPI_Flash_Read(buf,waddr[0],4);  
  for(i=0;i<4;i++)fwbuf[j++]=buf[i];
  goto loop2;
 }
 if(dll<=20)waddr[2]=H_rom+(dll-1)*6400+(daa-1)*32;
 if(dll>40&&dll<49)waddr[2]=0xd0000+(daa-1)*32;    									
 SPI_Flash_Read(buf,waddr[2],26);
 for(i=6;i<22;i++)  
		{if((buf[i])!=0xff) fwbuf[j++]=buf[i]; else fwbuf[j++]=0x20; }//×¢ÊÍÐÅÏ¢									
 nam=buf[0];flo=buf[1];zon=buf[2];	
 fwbuf[j++]=0x0a; //×ßÖ½										

 SPI_Flash_Read(buf,waddr[0],4);
		for(i=0;i<4;i++)fwbuf[j++]=buf[i]; //alarm
 fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ

 if(nam>0x80) nam=nam-0x40;  //LQL
 waddr[1]=H_nam+nam*8;
 SPI_Flash_Read(buf,waddr[1],8);
 for(i=0;i<8;i++)
		{if((buf[i])!=0xff) fwbuf[j++]=buf[i]; else fwbuf[j++]=0x20;}//dev_name

 fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ
 waddr[3]=H_zon+zon*6;   //·ÖÇø
 SPI_Flash_Read(buf,waddr[3],6);
 for(i=0;i<6;i++)
		{if((buf[i])!=0xff) fwbuf[j++]=buf[i];else fwbuf[j++]=0x20; }

 fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ
 if(flo>0&&flo<0xff)
 { if(flo>128) {flo=flo-128;fwbuf[j++]=0x2d;}//Â¥²ã
	 fwbuf[j++]=flo/10+0x30;fwbuf[j++]=flo%10+0x30;fwbuf[j++]=0x46;//F
 }
loop2:fwbuf[j++]=0x0a;//×ßÖ½
  fwbuf[j++]=0x4d;fwbuf[j++]=0x3d;
	if(area<=9){fwbuf[j++]=0x30+area/10;fwbuf[j++]=0x30+area%10;}else fwbuf[j++]=0x30;
	fwbuf[j++]=0x20;									
  fwbuf[j++]=0x4c;fwbuf[j++]=0x3d;
	if(dll<=48){fwbuf[j++]=0x30+dll/10;fwbuf[j++]=0x30+dll%10;}else fwbuf[j++]=0x30;
	fwbuf[j++]=0x20;//L=*
  fwbuf[j++]=0x41;fwbuf[j++]=0x3d;fwbuf[j++]=daa/100+0x30;fwbuf[j++]=(daa%100)/10+0x30;
	fwbuf[j++]=daa%10+0x30;fwbuf[j++]=0x20;//A=*
  fwbuf[j++]=ctime[4]/10+0x30;fwbuf[j++]=ctime[4]%10+0x30;fwbuf[j++]=0x2f;//Äê/ÔÂ/ÈÕ
  fwbuf[j++]=ctime[3]/10+0x30;fwbuf[j++]=ctime[3]%10+0x30;fwbuf[j++]=0x2f;
  fwbuf[j++]=ctime[2]/10+0x30;fwbuf[j++]=ctime[2]%10+0x30;fwbuf[j++]=0x20;
  fwbuf[j++]=ctime[1]/10+0x30;fwbuf[j++]=ctime[1]%10+0x30;fwbuf[j++]=0x3a;//Ê±:·Ö
  fwbuf[j++]=ctime[0]/10+0x30;fwbuf[j++]=ctime[0]%10+0x30;						
  fwbuf[j++]=0x0d;
  for(i=0;i<j;i++)  Printing(fwbuf[i]);
  return;
}



void Printing (u8 cde)
{ //u8 i;
  UART4_SendByte(cde);
  Delay16s(30,20); 
}

void DJBcx(u8 dwn)	//µÇ¼Ç±í²éÑ¯ djbaddr djbcount
 {u8 nam,flo,zon,buf[3];
  u16 pos,color;   //j=0,dloop,
	u8 count; 
  u32 waddr;
	djbfg++;
	pos=GZtop_y; color=Black;
	 
if((dwn==3)&&(djbloop<18))  //-> loop+1
{	djbloop+=1;djbaddr=1; djbcount=0;
  dwn=1;
}
if((dwn==4)&&(djbloop>1))  //<- loop-1
{	djbloop-=1;djbaddr=1; djbcount=0;
  dwn=1;
}
	 
if(dwn==2) //&&(djbaddr>5))  //UP
{	djbaddr=1; djbcount=0;
  dwn=1;
}
	 
if((dwn==1)&&(djbaddr<196))  //down
{	do       //Ã¿ÆÁÏÔÊ¾5Ìõ
	{	//if((i%2)==0)color=Blue;else color=Black;
		if((djb[djbloop-1][djbaddr]&0x80)==0x80)//ÏÔÊ¾µÇ¼ÇµÄ»ØÂ·µØÖ·
		{	
//USART1_SendByte(djbaddr); 		
			Dispsz16(djbcount+count+1,4,pos,nob_x,color,White);								//nob
			Dispsz24(djbloop,2,pos,adr_x,color,White);		//LL
			Dispsz24(djbaddr,3,pos,adr_x+34,color,White);	//AA
			waddr=0xa0000+(djbloop-1)*6400+(djbaddr-1)*32;
			SPI_Flash_Read(buf,waddr,3);
			nam=buf[0];flo=buf[1];zon=buf[2];
if(nam>0x80) nam=nam-0x40; //LQL			
			Disprom(H_nam,0,nam,pos,nam_x,color,White);
			if(zon<128&&zon>0)	Dispsz24(zon,2,pos,240,color,White);
			else Dispsz24(0,2,pos,240,color,White);     
              //Â¥²ã
			if(flo<0xff)
			{	if(flo>128)
				{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
					Dispsz24(flo,1,pos,flo_x+12,color,White);
				}
				else  Dispsz24(flo,2,pos,flo_x,color,White);
				Dispzf24('F',pos,320,color,White);
			}
			Disprom(H_rom,djbloop,djbaddr,pos,rom_x,color,White);//ÇøÓò·¿¼äÃû³Æ
			Disprom(H_zon,djbloop,zon,pos,zon_x,color,White); 					//zon
			count++; pos+=27;
//USART1_SendByte(count); Delay16s(16,2);
		}
		djbaddr++; 
//USART1_SendByte(djbaddr);	Delay16s(16,2);	
	}while ((djbaddr<200)&&(count<=4));	
	djbcount= djbcount+count;	
	if(djbaddr>200){djbaddr=1;djbloop++;}//µ¥¸ö»ØÂ·ÏÔÊ¾Íê±Ï						  
}
//if((dwn==2)) //&&(djbaddr>5))  //UP
//{	
//	djbaddr=1; djbcount=0;

	//	do       //Ã¿ÆÁÏÔÊ¾5Ìõ
//	{	//if((i%2)==0)color=Blue;else color=Black;
//		if((djb[djbloop-1][djbaddr-1]&0x80)==0x80)//ÏÔÊ¾µÇ¼ÇµÄ»ØÂ·µØÖ·
//		{	
//USART1_SendByte(djbaddr); 		
//			Dispsz16(djbcount+count+1,4,pos,nob_x,color,White);								//nob
//			Dispsz24(djbloop,2,pos,adr_x,color,White);		//LL
//			Dispsz24(djbaddr,3,pos,adr_x+34,color,White);	//AA
//			waddr=0xa0000+(djbloop-1)*6400+(djbaddr-1)*32;
//			SPI_Flash_Read(buf,waddr,3);
//			nam=buf[0];flo=buf[1];zon=buf[2];
//if(nam>0x80) nam=nam-0x40; //LQL			
//			Disprom(H_nam,0,nam,pos,nam_x,color,White);
//			if(zon<128&&zon>0)	Dispsz24(zon,2,pos,240,color,White);
//			else Dispsz24(0,2,pos,240,color,White);     
//              //Â¥²ã
//			if(flo<0xff)
//			{	if(flo>128)
//				{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
//					Dispsz24(flo,1,pos,flo_x+12,color,White);
//				}
//				else  Dispsz24(flo,2,pos,flo_x,color,White);
//				Dispzf24('F',pos,320,color,White);
//			}
//			Disprom(H_rom,djbloop,djbaddr,pos,rom_x,color,White);//ÇøÓò·¿¼äÃû³Æ
//			Disprom(H_zon,djbloop,zon,pos,zon_x,color,White); 					//zon
//			count++; pos+=27;
////USART1_SendByte(count); Delay16s(16,2);
//		}
//		djbaddr--; 
////USART1_SendByte(djbaddr);	Delay16s(16,2);	
//	}while ((djbaddr>=1)&&(count<=4));	
//	djbcount= djbcount-count;	
//	if(djbaddr<5){djbaddr=1;djbloop++;}//µ¥¸ö»ØÂ·ÏÔÊ¾Íê±Ï						  
//}	
	return;
}
 
void PBBcx(u8 dwn)  //ÆÁ±Î±í²éÑ¯ pbbaddr pbbcount
{	u8 nam,flo,zon,buf[3]; //time[5],
//	u8 num,dll,daa,
  u16 pos,color;   //j=0,dloop,
	u8 count; 
  u32 waddr;
	pbbfg++;
	pos=GZtop_y; color=Black;
	do       //Ã¿ÆÁÏÔÊ¾5Ìõ
	{	//if((i%2)==0)color=Blue;else color=Black;
//USART1_SendByte(djb[pbbloop-1][pbbaddr]); 		
		if((djb[pbbloop-1][pbbaddr]&0xc0)==0xc0)//ÏÔÊ¾µÇ¼ÇµÄ»ØÂ·µØÖ·
		{	Dispsz16(pbbcount+count+1,4,pos,nob_x,color,White);								//nob
			Dispsz24(pbbloop,2,pos,adr_x,color,White);		//LL
			Dispsz24(pbbaddr,3,pos,adr_x+34,color,White);	//AA
			waddr=0xa0000+(pbbloop-1)*6400+(pbbaddr-1)*32;
			SPI_Flash_Read(buf,waddr,3);
			nam=buf[0];flo=buf[1];zon=buf[2];
			Disprom(H_nam,0,nam,pos,nam_x,color,White);
			if(zon<128&&zon>0)	Dispsz24(zon,2,pos,240,color,White);
			else Dispsz24(0,2,pos,240,color,White);     
              //Â¥²ã
			if(flo<0xff)
			{	if(flo>128)
				{	flo=flo-128;Dispzf24('-',pos,flo_x,color,White);
					Dispsz24(flo,1,pos,flo_x+12,color,White);
				}
				else  Dispsz24(flo,2,pos,flo_x,color,White);
				Dispzf24('F',pos,320,color,White);
			}
			Disprom(H_rom,pbbloop,pbbaddr,pos,rom_x,color,White);//ÇøÓò·¿¼äÃû³Æ
			Disprom(H_zon,pbbloop,zon,pos,zon_x,color,White); 					//zon
			count++; pos+=27;
//USART1_SendByte(count); Delay16s(16,2);
		}
		pbbaddr++; 
//USART1_SendByte(pbbaddr);	Delay16s(16,2);	
	}while ((pbbaddr<200)&&(count<=4));	
	pbbcount= pbbcount+count;	
	if(pbbaddr>200){pbbaddr=1;pbbloop++;}//µ¥¸ö»ØÂ·ÏÔÊ¾Íê±Ï						  
	return;

//	pbbfg++;
// ClearS(192,0,800,288,White);
// num=xspbs-12*dwn;j=num-1;
// for(i=0;i<12;i++)
//   {Disprom(H_alm,0,32,24*i+192,0,White,Black);
//    dll=Disp_pb[j][1];daa=Disp_pb[j][2];
//		Dispsz24(dll,2,24*i+192,48,White,Black);Dispsz24(daa,3,24*i+192,80,White,Black); 
//		waddr=0xa0000+(dll-1)*6400+(daa-1)*32;
//    SPI_Flash_Read(buf,waddr,3);
//		nam=buf[0];flo=buf[1];zon=buf[2]; 	
//    Disprom(H_nam,0,nam,24*i+192,112,White,Black);
//		//·ÖÇø
//		if(zon<128&&zon>0)Dispsz24(zon,2,24*i+192,264,White,Black);else Dispsz24(0,2,24*i+192,264,White,Black);		
//	  //Â¥²ã
//    if(flo<0xff)
//			  {if(flo>128){flo=flo-128;Dispzf24('-',24*i+192,312,White,Black);Dispsz24(flo,1,24*i+192,328,White,Black);}
//			   else Dispsz24(flo,2,24*i+192,312,White,Black);
//			   Dispzf24('F',24*i+192,344,White,Black);
//			  }		 
//    Disprom(H_rom,dll,daa,24*i+192,368,White,Black);//ÇøÓò·¿¼äÃû³Æ		
//	  time[4]=Disp_pb[j][4];//Äê
//	  x=Disp_pb[j][5]>>5;y=Disp_pb[j][6]>>5;time[3]=x*8+y;//ÔÂ
//	  time[2]=Disp_pb[j][5]&0x1f;//ÈÕ
//	  time[1]=Disp_pb[j][6]&0x1f;//Ê±
//	  time[0]=Disp_pb[j][7];     //·Ö
//		Disptime(time+4,5,24*i+192,616,White,Black);
//    Dispsz16(num,3,24*i+192,744,White,Black); 
//    num--;if(num==0) break;	
//    j--;}
 } 
 

void GZcx(u8 dwn)		//¸ôÀë±í²éÑ¯
{u8 i,x,y,dll,daa,nam,flo,zon,time[5],buf[3];
 u16 j,num;  //dloop,
 u32 waddr;
 gzbfg++;
 ClearS(192,0,800,288,Yellow);
 num=gzzz-12*dwn;j=num-1;
 for(i=0;i<12;i++)
     {Disprom(H_alm,0,Disp_gz[j][0],24*i+192,0,Yellow,Black);
	    dll=Disp_gz[j][1];daa=Disp_gz[j][2];  //dloop=0;
      if(daa==0||Disp_gz[j][1]>=129) //»ØÂ·¹ÊÕÏ
          {if(dll<=8&&dll>0){Disprom(H_alm,0,65,24*i+192,112,Yellow,Black);Dispsz24(dll,4,24*i+192,48,Yellow,Black);}//»ØÂ·
           if(dll==129)Disprom(H_alm,0,66,24*i+192,112,Yellow,Black);//Ö÷µç
           if(dll==130)Disprom(H_alm,0,67,24*i+192,112,Yellow,Black);//±¸µç
					 if(dll==131)Disprom(H_alm,0,68,24*i+192,112,Yellow,Black);
           if(dll==134)Disprom(H_alm,0,70,24*i+192,112,Yellow,Black);
           goto loop;}							 
      else //Õý³£¼ÇÂ¼
		       if(dll>0&&dll<=16)//»ØÂ·Ì½²âÆ÷
					   {Dispsz24(dll,2,24*i+192,48,Yellow,Black);Dispsz24(daa,3,24*i+192,80,Yellow,Black);
							Disprom(H_rom,dll,daa,24*i+192,368,Yellow,Black);
							waddr=0xa0000+(dll-1)*6400+(daa-1)*32;
					   }
		     	 if(dll>40&&dll<49)
					   {Dispsz16(dll,2,24*i+192,48,Yellow,Black);Dispsz16(daa,3,24*i+192,64,Yellow,Black);
							Disprom(H_romzx,dll,daa,24*i+192,368,Yellow,Black);	
							waddr=0xd0000+(daa-1)*32;
						 }
      	   SPI_Flash_Read(buf,waddr,3);
			     nam=buf[0];flo=buf[1];zon=buf[2];
           Disprom(H_nam,0,nam,24*i+192,112,Yellow,Black);
			     //·ÖÇø
		        if(zon<128&&zon>0)Dispsz24(zon,2,24*i+192,264,Yellow,Black);else Dispsz24(0,2,24*i+192,264,Yellow,Black);		
			      //Â¥²ã
            if(flo<0xff)
			        {if(flo>128){flo=flo-128;Dispzf24('-',24*i+192,312,Yellow,Black);Dispsz24(flo,1,24*i+192,328,Yellow,Black);}
			         else Dispsz24(flo,2,24*i+192,312,Yellow,Black);
			         Dispzf24('F',24*i+192,344,Yellow,Black);
			        }		           
			loop:time[4]=Disp_gz[j][4];//Äê
   	       x=Disp_gz[j][5]>>5;y=Disp_gz[j][6]>>5;time[3]=x*8+y;//ÔÂ
	         time[2]=Disp_gz[j][5]&0x1f;//ÈÕ
	         time[1]=Disp_gz[j][6]&0x1f;//Ê±
	         time[0]=Disp_gz[j][7];     //·Ö
			     Disptime(time+4,5,24*i+192+4,616,Yellow,Black);
           Dispsz16(num,3,24*i+192,744,Yellow,Black);
           num--;if(num==0) break;
          j--;
      }
 }
			     
 
 
 
void LiandongQT(void)
{u8 i,k=0,dll,daa;
 u16 cb;
 cb=572+16*knum;k=shu[knum];
 if(k==0x0d)
 loop:{for(i=0;i<12;i++)shu[i]=0;knum=0; //ÍË¸ñ
       ClearXY(407,380,24,420,White);dstr=LDMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);
       return;} 		 
 if(k==0x13){Esc();return;}  //ÍË³ö
 Dispsz24(k,1,407,cb,White,Black);Dispzf('_',407,cb+16,White,Black);knum++;
 if(knum==5)
     {dll=shu[0]-1;daa=shu[1]*100+shu[2]*10+shu[3];
      dstr=LDMenu;
      if(shu[4]>1) goto loop;
      if(shu[4]==1)
         {	if((djb[dll][daa]&0xed)==0x81)//µÇ¼Ç,Î´Æô,ÎÞÆÁ¹Ê
            {	Dispstr(5,1,407,688,White,Black);djb[dll][daa]|=0x20;
							HJalarm(3,0,dll+1,daa);
							U7_QDHDcmd(4,(dll)*256+daa);
							goto loop1;
						}
						else goto loop;
				 }
      if(shu[4]==0)
         {	if((djb[dll][daa]&0xe5)==0xa1)
            {	Dispstr(8,1,407,688,White,Black);djb[dll][daa]&=0xdf;
							HJalarm(4,0,dll+1,daa);
//USART1_SendByte(0xfc);
							U7_QDHDcmd(4,(dll)*256+daa);
						}
						else goto loop;}
loop1:ldfg++;for(i=0;i<10;i++)shu[i]=0;knum=0;MenuN=0;
      return;}
 } 
 
 
 
void Geli(void)
{u8 i,k=0,dll,daa;
 u16 cb;
 cb=572+16*knum;k=shu[knum];
 if(k==0x0d)
 loop:{for(i=0;i<12;i++)shu[i]=0;knum=0; //ÍË¸ñ
       ClearXY(407,380,24,420,White);dstr=GLMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);
       return;} 		 
 if(k==0x13){Esc();return;}  //ÍË³ö
 Dispsz24(k,1,407,cb,White,Black);Dispzf('_',407,cb+16,White,Black);knum++;
 if(knum==5)
     {dll=shu[0]-1;daa=shu[1]*100+shu[2]*10+shu[3];
      dstr=GLMenu;
      if(shu[4]>1) goto loop;
      if(shu[4]==1)
         {if((djb[dll][daa]&0xc0)==0x80)
            {Dispstr(5,1,407,688,White,Black);djb[dll][daa]|=0x40;HJalarm(32,0,dll+1,daa);goto loop1;}
          else goto loop;}
      if(shu[4]==0)
         {if((djb[dll][daa]&0xc0)==0xc0)
            {Dispstr(8,1,407,688,White,Black);djb[dll][daa]&=0xbf;HJalarm(33,0,dll+1,daa);}
          else goto loop;}
loop1:glfg++;for(i=0;i<10;i++)shu[i]=0;knum=0;MenuN=0;
      return;}
 }





void Displine(u8 type,u8 dwn)
{u8 i,k,x,y,alm,dll,daa,bufjl[96],buf[3],time[5],nam,flo,zon;//area,4-year,3-mon,2-day,1-hour,0-min;
 u16 zz,zztrue,num,color,bcolor; //,u8dloop=0,
 u32 waddr,addr;
 for(i=0;i<96;i++)bufjl[i]=0;
 if(type==1){waddr=0x80000;zz=hjjlzz;zztrue=hjjltrue;color=Red;hjjlfg++;}
 if(type==2){waddr=0x84000;zz=ldjlzz;zztrue=ldjltrue;color=Blue;ldjlfg++;}
 if(type==3){waddr=0x88000;zz=tjlzz;zztrue=tjltrue;color=Yellow;qtjlfg++;}
 num=zz-12*dwn;
 if(num>=12){k=12;addr=waddr+zztrue*8-96-96*dwn;}
 else{k=num%12;addr=waddr+zztrue*8-k*8-96*dwn; }
 SPI_Flash_Read(bufjl,addr,k*8);  
 ClearS(192,0,800,288,color);
 for(i=0;i<k;i++)
      {if((i%2)==0)bcolor=Black;else bcolor=Blue;
	    // dloop=0;
       alm=bufjl[8*(k-i-1)];Disprom(H_alm,0,alm,24*i+192,0,color,bcolor);
	     dll=bufjl[8*(k-i-1)+1];daa=bufjl[8*(k-i-1)+2];
//				area=bufjl[8*(k-i-1)+3];
       if(daa==0||dll>=129)//ÌØÊâ¼ÇÂ¼
            {if(dll<=8&&dll>0){Disprom(H_alm,0,65,24*i+192,112,color,bcolor);Dispsz24(dll,4,24*i+192,48,color,bcolor);}//»ØÂ· 
             if(dll==129)Disprom(H_alm,0,66,24*i+192,112,color,bcolor);//Ö÷µç
             if(dll==130)Disprom(H_alm,0,67,24*i+192,112,color,bcolor);//±¸µç
						 if(dll==131)Disprom(H_alm,0,68,24*i+192,112,color,bcolor);
             if(dll==134)Disprom(H_alm,0,70,24*i+192,112,color,bcolor);				
						 if(dll==151)Disprom(H_alm,0,81,24*i+192,112,color,bcolor);
						 if(dll==152)Disprom(H_alm,0,82,24*i+192,112,color,bcolor);
             goto loop;}							 
       else //Õý³£¼ÇÂ¼
            if(dll>0&&dll<=16){Dispsz24(dll,2,24*i+192,48,color,bcolor);Dispsz24(daa,3,24*i+192,80,color,bcolor);     
							                 Disprom(H_rom,dll,daa,24*i+192,368,color,bcolor);
			                         waddr=0xa0000+(dll-1)*6400+(daa-1)*32;}
						if(dll>40&&dll<48){Dispsz16(dll,2,24*i+192,48,color,bcolor);Dispsz16(daa,3,24*i+192,64,color,bcolor);
							                 Disprom(H_romzx,dll,daa,24*i+192,368,color,bcolor);	
			                         waddr=0xd0000+(daa-1)*32;}
			      SPI_Flash_Read(buf,waddr,3);
			      nam=buf[0];flo=buf[1];zon=buf[2];
			      Disprom(H_nam,0,nam,24*i+192,112,color,bcolor);
			      //·ÖÇø
		        if(zon<128&&zon>0)Dispsz24(zon,2,24*i+192,264,color,bcolor);else Dispsz24(0,2,24*i+192,264,color,bcolor);		
			      //Â¥²ã
            if(flo<0xff)
			        {if(flo>128){flo=flo-128;Dispzf24('-',24*i+192,312,color,bcolor);Dispsz24(flo,1,24*i+192,328,color,bcolor);}
			         else Dispsz24(flo,2,24*i+192,312,color,bcolor);
			         Dispzf24('F',24*i+192,344,color,bcolor);
			        }		 
  loop:time[4]=bufjl[8*(k-i-1)+4];//Äê
			 x=bufjl[8*(k-i-1)+5]>>5;y=bufjl[8*(k-i-1)+6]>>5;time[3]=x*8+y;//ÔÂ
			 time[2]=bufjl[8*(k-i-1)+5]&0x1f;//ÈÕ
			 time[1]=bufjl[8*(k-i-1)+6]&0x1f;//Ê±
			 time[0]=bufjl[8*(k-i-1)+7];     //·Ö
       Disptime(time+4,5,24*i+192+4,616,color,bcolor);
       Dispsz16(num,4,24*i+192,744,color,bcolor);
       num--;
       if(num==0){jlend=1;break;}//ÏÔÊ¾Íê±Ï
      }
}

void Displine_JLB(u8 type,u8 dwn)
{	u8 i,k,x,y,alm,dll,daa,bufjl[96],buf[3],time[5],nam,flo,zon;//area,4-year,3-mon,2-day,1-hour,0-min;
	u16 zz,zztrue,num,color,bcolor; //,u8dloop=0,
	u32 waddr,addr;
//USART1_SendByte(type);
	if(type==0){	return;}  //Esc(); return;
	for(i=0;i<96;i++)bufjl[i]=0;
	if(type==1){waddr=0x80000;zz=hjjlzz;zztrue=hjjltrue;hjjlfg++;color=Black;}//Red
	if(type==2){waddr=0x84000;zz=ldjlzz;zztrue=ldjltrue;ldjlfg++;color=Black;}//Blue
	if(type==3){waddr=0x88000;zz=tjlzz;zztrue=tjltrue;qtjlfg++;color=Black;}//Yellow
	num=zz-5*dwn;
	if(num>=5){k=5;addr=waddr+zztrue*8-5*8-5*8*dwn;}
	else{k=num%5;addr=waddr+zztrue*8-k*8-5*8*dwn; }
	/*¸Ä
	if(num>=5){k=1;addr=waddr+zztrue*8-5*8-5*8*dwn;}
	else{k=num%1;addr=waddr+zztrue*8-k*8-5*8*dwn; }
	*/
	SPI_Flash_Read(bufjl,addr,k*8);  
// ClearS(192,0,800,288,color);
	for(i=0;i<k;i++)
	{	bcolor=White;
		//if((i%2)==0)bcolor=Black;else bcolor=Blue;	    // dloop=0;
		alm=bufjl[8*(k-i-1)];
		Disprom(H_alm,0,alm,27*i+GZtop_y,alm_x,color,bcolor);
		dll=bufjl[8*(k-i-1)+1];daa=bufjl[8*(k-i-1)+2];
		//area=bufjl[8*(k-i-1)+3];
		if(daa==0||dll>=129)//ÌØÊâ¼ÇÂ¼
		{	if(dll<=16&&dll>0) //»ØÂ· 
			{	Disprom(H_alm,0,65,27*i+GZtop_y,alm_x,color,bcolor);
				Dispsz24(dll,4,27*i+GZtop_y,adr_x,color,bcolor);
			}
			if(dll==129)Disprom(H_alm,0,66,27*i+GZtop_y,alm_x,color,bcolor);//Ö÷µç
			if(dll==130)Disprom(H_alm,0,67,27*i+GZtop_y,alm_x,color,bcolor);//±¸µç
			if(dll==131)Disprom(H_alm,0,68,27*i+GZtop_y,alm_x,color,bcolor);
			if(dll==134)Disprom(H_alm,0,70,27*i+GZtop_y,alm_x,color,bcolor);				
			if(dll==151)Disprom(H_alm,0,81,27*i+GZtop_y,alm_x,color,bcolor);
			if(dll==152)Disprom(H_alm,0,82,27*i+GZtop_y,alm_x,color,bcolor);
			goto loop;
		}							 
		else //Õý³£¼ÇÂ¼
		{	if(dll>0&&dll<=16)
			{	Dispsz24(dll,2,27*i+GZtop_y,adr_x,color,bcolor);
				Dispsz24(daa,3,27*i+GZtop_y,adr_x+34,color,bcolor);     
				Disprom(H_rom,dll,daa,27*i+GZtop_y,rom_x,color,bcolor);
				waddr=0xa0000+(dll-1)*6400+(daa-1)*32;
			}
			if(dll>40&&dll<48)
			{	Dispsz16(dll,2,27*i+GZtop_y,adr_x,color,bcolor);
				Dispsz16(daa,3,27*i+GZtop_y,x+34,color,bcolor);
				Disprom(H_romzx,dll,daa,24*i+GZtop_y,alm_x,color,bcolor);	
				waddr=0xd0000+(daa-1)*32;
			}
		}
		SPI_Flash_Read(buf,waddr,3);
		nam=buf[0];flo=buf[1];zon=buf[2];
if(nam>0x80) nam=nam-0x40; //LQL
//USART1_SendByte(nam);	Delay16s(16,2);		
		Disprom(H_nam,0,nam,27*i+GZtop_y,nam_x,color,bcolor);
		Disprom(H_zon,0,zon,27*i+GZtop_y,zon_x,color,White); 		//·ÖÇø
//		if(zon<128&&zon>0)	Dispsz24(zon,2,27*i+GZtop_y,264,color,bcolor);
//		else 								Dispsz24(0,2,27*i+GZtop_y,264,color,bcolor);		//·ÖÇø
		if(flo<0xff)	//Â¥²ã
		{	if(flo>128)
			{	flo=flo-128;Dispzf24('-',27*i+GZtop_y,flo_x,color,bcolor);
				Dispsz24(flo,1,27*i+GZtop_y,flo_x+12,color,bcolor);
			}
			else {	Dispsz24(flo,2,27*i+GZtop_y,flo_x,color,bcolor);}
//		Dispzf24('F',27*i+GZtop_y,344,color,bcolor);
		}		 
loop:	
		time[4]=bufjl[8*(k-i-1)+4];//Äê
		x=bufjl[8*(k-i-1)+5]>>5;y=bufjl[8*(k-i-1)+6]>>5;time[3]=x*8+y;//ÔÂ
		time[2]=bufjl[8*(k-i-1)+5]&0x1f;//ÈÕ
		time[1]=bufjl[8*(k-i-1)+6]&0x1f;//Ê±
		time[0]=bufjl[8*(k-i-1)+7];     //·Ö
		Disptime(time+4,5,27*i+GZtop_y+4,tim_x,color,bcolor);
		Dispsz16(num,4,27*i+GZtop_y,nob_x,color,bcolor);
		num--;
		if(num==0)
		{jlend=1;	break;}//ÏÔÊ¾Íê±Ï
	}
}

/*ÐÂÔö*/
//Éè¼ÆÊýÁ¿Í³¼Æ
void designCntStat(void){
	int i;
	u8 buf[32];
	
	//³õÊ¼»¯
	for(i=0;i<DesignCntArray_n;i++){
		DesignCntArray[i] = 0;
	}
	//±éÀúÍ³¼ÆÀàÐÍÊýÁ¿
	for(i=0;i<4096;i++){
		SPI_Flash_Read(buf,0xa0000+(0x20*i),0x20);
		//´æÔÚµãÎ»ÐÅÏ¢
		if(buf[0]!=0xFF){
			//ÊäÈëÉè±¸
			if(buf[0]<0x64){
				//¸ÐÑÌÌ½²âÊýÁ¿
				if(buf[0]==0x01){DesignCntArray[0]++;}
				//¸ÐÎÂÌ½²âÊýÁ¿
				else if(buf[0]==0x02){DesignCntArray[1]++;}
				//ÊÖ¶¯±¨¾¯ÊýÁ¿
				else if(buf[0]==0x03){DesignCntArray[2]++;}
				//Ïû»ðË¨Å¥ÊýÁ¿
				else if(buf[0]==0x04){DesignCntArray[3]++;}
				//ÆäËû±¨¾¯ÊýÁ¿
				else{
					DesignCntArray[4]++;
				}
			}
			//Êä³öÉè±¸
			else if(buf[0]>=0x64){
				//Ïû·À±ÃÊýÁ¿
				if(buf[0]==0x81){DesignCntArray[5]++;}
				//ÅçÁÜ±ÃÊýÁ¿
				else if(buf[0]==0x82){DesignCntArray[6]++;}
				//ÅÅÑÌ»úÊýÁ¿
				else if(buf[0]==0x83){DesignCntArray[7]++;}
				//ËÍ·ç»úÊýÁ¿
				else if(buf[0]==0x84){DesignCntArray[8]++;}
				//µçÌÝÆÈ½µÊýÁ¿
				else if(buf[0]==0x85){DesignCntArray[9]++;}
				//Ó¦¼±¹ã²¥ÊýÁ¿
				else if(buf[0]==0x86){DesignCntArray[10]++;}
				//·À»ð¾íÁ±ÊýÁ¿
				else if(buf[0]==0x87){DesignCntArray[11]++;}
				//ÆäËûÊä³ö
				else{
					DesignCntArray[12]++;
				}
			}
		}
		//¸ÃµãÎ»ÎÞÐÅÏ¢
		else{}
	}
}
//Õý³£¹¤×÷Êý×ÔÔöÍ³¼Æ
void NormalWorkStatInc(u8 loop,u8 addr){
	u8 i;
	u8 buf[32];
	
	//¶ÁÈ¡¸ÃµãÎ»µÄÐÅÏ¢
	SPI_Flash_Read(buf,0xa0000+loop*6400+(addr-1)*32,0x20);
	
	//´æÔÚµãÎ»ÐÅÏ¢
	if(buf[0]!=0xFF){
		//ÊäÈëÉè±¸
		if(buf[0]<0x64){
			//¸ÐÑÌÌ½²âÊýÁ¿
			if(buf[0]==0x01){NormalWorkCntArray[0]++;}
			//¸ÐÎÂÌ½²âÊýÁ¿
			else if(buf[0]==0x02){NormalWorkCntArray[1]++;}
			//ÊÖ¶¯±¨¾¯ÊýÁ¿
			else if(buf[0]==0x03){NormalWorkCntArray[2]++;}
			//Ïû»ðË¨Å¥ÊýÁ¿
			else if(buf[0]==0x04){NormalWorkCntArray[3]++;}
			//ÆäËû±¨¾¯ÊýÁ¿
			else{
				NormalWorkCntArray[4]++;
			}
		}
		//Êä³öÉè±¸
		else if(buf[0]>=0x64){
			//Ïû·À±ÃÊýÁ¿
			if(buf[0]==0x81){NormalWorkCntArray[5]++;}
			//ÅçÁÜ±ÃÊýÁ¿
			else if(buf[0]==0x82){NormalWorkCntArray[6]++;}
			//ÅÅÑÌ»úÊýÁ¿
			else if(buf[0]==0x83){NormalWorkCntArray[7]++;}
			//ËÍ·ç»úÊýÁ¿
			else if(buf[0]==0x84){NormalWorkCntArray[8]++;}
			//µçÌÝÆÈ½µÊýÁ¿
			else if(buf[0]==0x85){NormalWorkCntArray[9]++;}
			//Ó¦¼±¹ã²¥ÊýÁ¿
			else if(buf[0]==0x86){NormalWorkCntArray[10]++;}
			//·À»ð¾íÁ±ÊýÁ¿
			else if(buf[0]==0x87){NormalWorkCntArray[11]++;}
			//ÆäËûÊä³ö
			else{
				NormalWorkCntArray[12]++;
			}
		}
	}
	//¸ÃµãÎ»ÎÞÐÅÏ¢
	else{}
}

//¹ÊÕÏÊý×ÔÔöÍ³¼Æ
void BreakDownStatInc(u8 loop,u8 addr){
	u8 i;
	
	u8 buf[32];
	//¶ÁÈ¡¸ÃµãÎ»µÄÐÅÏ¢
	SPI_Flash_Read(buf,0xa0000+loop*6400+(addr-1)*32,0x20);
	
	//´æÔÚµãÎ»ÐÅÏ¢
	if(buf[0]!=0xFF){
		
		//ÊäÈëÉè±¸
		if(buf[0]<0x64){
			//¸ÐÑÌÌ½²âÊýÁ¿
			if(buf[0]==0x01){BreakdownCntArray[0]++;}
			//¸ÐÎÂÌ½²âÊýÁ¿
			else if(buf[0]==0x02){BreakdownCntArray[1]++;}
			//ÊÖ¶¯±¨¾¯ÊýÁ¿
			else if(buf[0]==0x03){BreakdownCntArray[2]++;}
			//Ïû»ðË¨Å¥ÊýÁ¿
			else if(buf[0]==0x04){BreakdownCntArray[3]++;}
			//ÆäËû±¨¾¯ÊýÁ¿
			else{
				BreakdownCntArray[4]++;
			}
		}
		//Êä³öÉè±¸
		else if(buf[0]>=0x64){
			//Ïû·À±ÃÊýÁ¿
			if(buf[0]==0x81){BreakdownCntArray[5]++;}
			//ÅçÁÜ±ÃÊýÁ¿
			else if(buf[0]==0x82){BreakdownCntArray[6]++;}
			//ÅÅÑÌ»úÊýÁ¿
			else if(buf[0]==0x83){BreakdownCntArray[7]++;}
			//ËÍ·ç»úÊýÁ¿
			else if(buf[0]==0x84){BreakdownCntArray[8]++;}
			//µçÌÝÆÈ½µÊýÁ¿
			else if(buf[0]==0x85){BreakdownCntArray[9]++;}
			//Ó¦¼±¹ã²¥ÊýÁ¿
			else if(buf[0]==0x86){BreakdownCntArray[10]++;}
			//·À»ð¾íÁ±ÊýÁ¿
			else if(buf[0]==0x87){BreakdownCntArray[11]++;}
			//ÆäËûÊä³ö
			else{
				BreakdownCntArray[12]++;
			}
		}
	}
	//¸ÃµãÎ»ÎÞÐÅÏ¢
	else{}
	
}
//ÆÁ±ÎÊý×ÔÔöÍ³¼Æ
void ShieldStatInc(u8 loop,u8 addr){
	u8 buf[32];
	//¶ÁÈ¡¸ÃµãÎ»µÄÐÅÏ¢
	SPI_Flash_Read(buf,0xa0000+loop*6400+(addr-1)*32,0x20);
	//´æÔÚµãÎ»ÐÅÏ¢
	if(buf[0]!=0xFF){
		//ÊäÈëÉè±¸
		if(buf[0]<0x64){
			//¸ÐÑÌÌ½²âÊýÁ¿
			if(buf[0]==0x01){ShieldCntArray[0]++;}
			//¸ÐÎÂÌ½²âÊýÁ¿
			else if(buf[0]==0x02){ShieldCntArray[1]++;}
			//ÊÖ¶¯±¨¾¯ÊýÁ¿
			else if(buf[0]==0x03){ShieldCntArray[2]++;}
			//Ïû»ðË¨Å¥ÊýÁ¿
			else if(buf[0]==0x04){ShieldCntArray[3]++;}
			//ÆäËû±¨¾¯ÊýÁ¿
			else{
				ShieldCntArray[4]++;
			}
		}
		//Êä³öÉè±¸
		else if(buf[0]>=0x64){
			//Ïû·À±ÃÊýÁ¿
			if(buf[0]==0x81){ShieldCntArray[5]++;}
			//ÅçÁÜ±ÃÊýÁ¿
			else if(buf[0]==0x82){ShieldCntArray[6]++;}
			//ÅÅÑÌ»úÊýÁ¿
			else if(buf[0]==0x83){ShieldCntArray[7]++;}
			//ËÍ·ç»úÊýÁ¿
			else if(buf[0]==0x84){ShieldCntArray[8]++;}
			//µçÌÝÆÈ½µÊýÁ¿
			else if(buf[0]==0x85){ShieldCntArray[9]++;}
			//Ó¦¼±¹ã²¥ÊýÁ¿
			else if(buf[0]==0x86){ShieldCntArray[10]++;}
			//·À»ð¾íÁ±ÊýÁ¿
			else if(buf[0]==0x87){ShieldCntArray[11]++;}
			//ÆäËûÊä³ö
			else{
				ShieldCntArray[12]++;
			}
		}
	}
	//¸ÃµãÎ»ÎÞÐÅÏ¢
	else{}
}
//¼ì²é¼ÇÂ¼±íÏÔÊ¾
void DispCheck_JLB(int dwn){
	int i,loop,addr;
	for(i=0; i<9;i++){
		//ÀàÐÍÃû³ÆÏÔÊ¾
		dstr=typeNameMenu;
		ClearXY(27*i+LDtop_y+27,nob_x,26,670,Black);
		Dispstr((TypeMenu[dwn]+i)*8,4,27*i+LDtop_y+27,type_x,Black,White);
		//Éè¼ÆÊýÁ¿ÏÔÊ¾
		Dispsz24(DesignCntArray[dwn+i],4,27*i+LDtop_y+27,DesignCnt_x+24,Black,White);
		//Õý³£¹¤×÷Êý
		Dispsz24(NormalWorkCntArray[dwn+i],4,27*i+LDtop_y+27,NormalCnt_x+24,Black,White);
		//¹ÊÕÏÊý
		Dispsz24(BreakdownCntArray[dwn+i],3,27*i+LDtop_y+27,BreakdownCnt_x+12,Black,White);
		//ÆÁ±ÎÊý
		Dispsz24(ShieldCntArray[dwn+i],3,27*i+LDtop_y+27,ShieldCnt_x+12,Black,White);
	}
}
void DengjiZD(u8 dcon,u8 dll,u8 daa)
{u8 i,j,k,t,dat,sum=0,buf[30];  //m,n,x,y,
 u16 sumz=0,z;
 u32 waddr;
//USART1_SendByte(dcon);	Delay16s(16,2);
//USART1_SendByte(dll);	Delay16s(16,2);
//USART1_SendByte(daa);	Delay16s(16,2);	
	Disphzs(0xb5c7,TiShi_y,TiShi_x,Blue,White);
	Disphzs(0xbcc7,TiShi_y,TiShi_x+16,Blue,White);
	Logit(35,dcon,dll,daa);	
	
if((dcon==1)&&(dll==0)&&(daa==0))
{	iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
	prlong3=0;

	LDGBstrb[2]=0; LDGBstrb[3]=0x60;	LDGBstrb[4]=0;
//	ur2_len=12; ur2_cmd=0; ur2_subnum=LDGBstrb[2];ur2_first=0xfd; ur2_last=0xfe; 	 //½ÓÊÕ°ü²ÎÊý
	Ur485_1=1;
	for(i=0;i<12;i++)			// from 1 to LEN
	{	sum=LDGBstrb[i];			//·¢ËÍLDGBÊý¾Ý
//USART1_SendByte(su);			
		USART2->DR = (sum & (uint16_t)0x01FF);
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
	}
	Delay16s(60,200); //400us
	Ur485_1=0;

	for(i=0;i<17;i++){for(j=0;j<202;j++){hjjs[i][j]=0;gzjs[i][j]=0;djb[i][j]=0;}}lpcxsum=0;	
	for(i=0;i<100;i++){for(j=0;j<6;j++) strblp[i][j]=0;} 
	SPI_Flash_Erase_Sector(141); SPI_Flash_Erase_Sector(142); SPI_Flash_Erase_Sector(143);
	xspbs=0;LedPB_off();
// Dispsz24(20,2,407,428,White,Magenta);Dispzf24(0x25,407,460,White,Magenta); //Íê³É20%
	Dispsz16(0,3,TiShi_y,TiShi_x+40,Blue,White); //0%

 //»ØÂ·µÇ¼Ç
	for(k=0;k<10;k++)
	{	for(t=0;t<16;t++)
		{	
			inpzz3=0;inpzz=0;lpfresh=0;
			Usart_loop3(t);
      do{Delay16s(50,20);}while(lpfresh<4);//400msÑ²¼ìÒ»´Î
		}
		Usart_loop7ZhiQi(2);		//ZQP Loop
//USART1_SendByte(U7_rxKB[2][0]);	
		for(t=0;t<32;t++)				//ZQP Online gzjs[16][0-31] read_sys is 1-32
		{	dat=U7_rxKB[2][t];
			if(((dat>>4)&0x01)==0x01)	gzjs[16][t*2]++;
			if((dat&0x01)==0x01)			gzjs[16][t*2+1]++;
		}
//USART1_SendByte(gzjs[16][0]);Delay16s(16,2);	
	  for(i=0;i<16;i++)
		{	if((lpxz_val[i][0]&0xfe)==0x20)
			{	gzjs[i][0]++;
				for(j=1;j<201;j++)
				{	if(lpxz_val[i][j]>=0x04)	gzjs[i][j]++;
				}
			}
		} 
	Dispsz16((k+1)*10,4,TiShi_y,TiShi_x+40,Blue,White); //0-100%
	}
//Ð´ÈëµÇ¼Ç±í
//USART1_SendByte(gzjs[0][1]); Delay16s(16,2);USART1_SendByte(gzjs[1][3]); Delay16s(16,2);
	for(i=0;i<16;i++)			//´óÓÚ4´ÎÕý³£µÇ¼Ç 0-15:Loop 16:ZQ
	{	if(gzjs[i][0]>=4)
		{	djb[i][0]=0x20;
			for(j=1;j<=201;j++)
			{	if(gzjs[i][j]>=4)
				{	waddr=0xa0000+i*6400+(j-1)*32;
					SPI_Flash_Read(buf,waddr,1);  
					k=buf[0];
if(k>0x80) k=k-0x40;  //LQL
//USART1_SendByte(k);	Delay16s(16,2);
					if(k<=64)  djb[i][j]=0x80; //if(k<=64||k==125)ÊäÈëÉè±¸
//					else{if(k==126)   {lpcxsum++;djb[i][j]=0x90;}
					else		   djb[i][j]=0x81;             //=0x81Êä³öÉè±¸
//					if(dcon==2)	djb[i][j]=0x00;            //Çå³ý				
				}
        sum++;
			}
			sumz=sumz+sum;
			djb[i][201]=sum;sum=0;
//			if(sumz==llpp)goto loop1; //point_limtd
		}
    else 
		{	for(j=0;j<=200;j++)djb[i][j]=0;djb[i][201]=0;}
	}
	for(i=0;i<32;i++)			//´óÓÚ4´ÎÕý³£µÇ¼Ç ZhiQi  DJB 16Lp:00-31
	{	if(gzjs[16][i]>=4) 
		{	djb[16][i]=0x81; }
		else
		{	djb[16][i]=0; }			
	}
//USART1_SendByte(djb[16][0]);
	for(z=0;z<4096;z++) downb[z]=0;
	for(i=0;i<16;i++)
	{	for(sum=0;sum<202;sum++)
			downb[i*256+sum]=djb[i][sum];
	}
	for(sum=0;sum<32;sum++)							//ZhiQi  16Lp_216-248:ZQ_0-31 read_sys is 1-32
	{	downb[15*256+216+sum]=djb[16][sum]; }
//USART1_SendByte(downb[15*256+216]);	Delay16s(16,2);
	SPI_Flash_Write_NoCheck(downb,0x8d000,4096);//Ð´Èë»ØÂ·µÇ¼Ç±í
	for(j=0;j<32;j++)		gzjs[16][j]=0;
	for(i=0;i<16;i++){	for(j=0;j<202;j++)gzjs[i][j]=0;}
}
 if((dll!=0)&&(daa!=0))	//single addr
 {	if((dcon==1)||(dcon==2))
	{	waddr=0xa0000+dll*6400+(daa-1)*32;
		SPI_Flash_Read(buf,waddr,1);  
		k=buf[0];
		if(k>0x80) k=k-0x40;  //LQL
//USART1_SendByte(k);	Delay16s(16,2);
		if(dcon==1)
		{	if(k<=64)  t=0x80; //if(k<=64||k==125)ÊäÈëÉè±¸
			else		   t=0x81;             //=0x81Êä³öÉè±¸ 
			sumz++;
		}	
		if(dcon==2)	
		{	t=0x00;  sumz--;          //Çå³ý		
		}
		buf[0]=t; waddr=0x8d000+(dll-1)*256+daa;
		SPI_Flash_Write(buf,waddr,1);  //
//USART1_SendByte(t);	Delay16s(16,2);
	}
 }

	Read_sys();
	Esc();
 	fk8save=0;	fk4save=3;	DispMenu(0,0);							//????
	DispMenu(1,0);	DispMenu(2,0);	DispMenu(3,0);	DispMenu(4,0);
	Disphzs(0xcdea,TiShi_y,TiShi_x,Blue,White);
	Disphzs(0xb3c9,TiShi_y,TiShi_x+16,Blue,White);

}
 
void DengjiSD(void)
{u8 i,k,dll,daa,buf[2],equ;
 u16 cb,j,sumz;
 u32 waddr;
 cb=572+16*knum;k=shu[knum];
 if(k==0x0d)
loop:{for(i=0;i<12;i++)shu[i]=0;knum=0; //ÍË¸ñ
	    ClearXY(407,380,24,420,White);dstr=DJMenu;Dispstr(0,6,407,380,White,Black);Dispzf('_',407,572,White,Black);
      return;}
 if(k==0x13){Esc();return;}//ÍË³ö
 Dispsz24(k,1,407,cb,White,Black);Dispzf('_',407,cb+16,White,Black);knum++;
 if(knum==6)
    {dll=(shu[0]-1)*10+shu[1];daa=shu[2]*100+shu[3]*10+shu[4];
     if(shu[5]==1)
         {waddr=0xa0000+dll*6400+(daa-1)*32;SPI_Flash_Read(buf,waddr,1);
					i=buf[0];
		      if(i<=64||i==125)equ=0x80; //Ì½²âÆ÷
          else{if(i==126)  equ=0x90; //²ãÏÔ
							 else        equ=0x81; //Êä³öÉè±¸
						  }							 					 
		      if(djb[dll][daa]<0x80)
						 {sumz=0;for(i=0;i<8;i++)sumz+=djb[i][201];
              if(sumz>=counter){dstr=DJMenu;Dispstr(14,2,407,636,White,Black);return;}
							else{dstr=DJMenu;Dispstr(5,1,407,636,White,Black);djb[dll][201]++;
								   if(equ==0x90)lpcxsum++;//Î´µÇ¼Ç,×ÜÊý¼Ó1 
								  }
						 }
          else{if(djb[dll][daa]!=equ){dstr=DLMenu;Dispstr(7,1,407,636,White,Black); 
						                          if(djb[dll][daa]!=0x90&&equ==0x90)lpcxsum++;
                                      if(djb[dll][daa]==0x90&&equ!=0x90)lpcxsum--;
						                         }
						  }
		      djb[dll][daa]=equ;		 
          goto loop1;}
       if(shu[5]==0){if(djb[dll][daa]>=0x80)
					                {sumz--;djb[dll][201]--;if(djb[dll][daa]==0x90)lpcxsum--;
													 djb[dll][daa]=0;dstr=DJMenu;Dispstr(8,1,407,636,White,Black);			
			                     goto loop1;}
			              }
        goto loop;
loop1:SPI_Flash_Read(downb,0x8d000,4096);//Ð´ÈëFlashÄÚ²¿µÇ¼Ç±íÖÐ 
      downb[dll*256+daa]=djb[dll][daa];downb[dll*256+201]=djb[dll][201];	
      SPI_Flash_Erase_Sector(141);
      SPI_Flash_Write_NoCheck(downb,0x8d000,4096);					
			if(xspbs>0){Modify(32,dll,daa);
                  for(i=0;i<xspbs;i++){for(k=0;k<8;k++)downb[i*8+k]=Disp_pb[i][k];}
                  downb[4095]=xspbs;
								 }
//			if(xspbs==0) LedPB_off(); else LedPB_on();  ???error
	    SPI_Flash_Erase_Sector(143);
      SPI_Flash_Write_NoCheck(downb,0x8f000,4096);
//loop3:
			if(strblp[dll][0]!=0xF0)
      {
        for(i=0;i<8;i++) //µÚÒ»Ñ­»·»ð¾¯²éÑ¯°ü     
    	  { if(djb[i][201]>0)
					{strblp[i][0]=0xF0;strblp[i][1]=0x01;strblp[i][2]=0xF2;strblp[i][5]=i;
			     strblp[i*10+8][0]=0xF0;strblp[i*10+8][1]=0x04;
					 strblp[i*10+8][2]=0xF2;strblp[i*10+8][5]=i;              
	         strblp[i*10+9][0]=0xF0;strblp[i*10+9][1]=0x03;
					 strblp[i*10+9][2]=0;strblp[i*10+9][3]=0xF2;strblp[i*10+9][5]=i;
		      }
          else    {strblp[i][0]=0;strblp[i][1]=0;strblp[i][2]=0;strblp[i][5]=0;}		
		    }
        for(i=1;i<8;i++)
        { for(j=0;j<8;j++)
		      { strblp[10*i+j][0]=strblp[j][0];strblp[10*i+j][1]=strblp[j][1];
					  strblp[10*i+j][2]=strblp[j][2];strblp[10*i+j][5]=strblp[j][5];
					}
		    }	
		
//        dscycle=33;hjcycle=10;gzcycle=60;								 
      }								 										 														 
//loop2:
      djfg++;for(i=0;i<10;i++)shu[i]=0;knum=0;MenuN=0;
  }
return;
} 
   
 

 
void Setup(void)
{u8 i,k,buf[32];u16 curh;
 if(knum==0)curh=428;
 if(knum==1)curh=504;
 if(knum==2)curh=580;
 k=shu[knum];
 if(k==0x0d){for(i=0;i<12;i++)shu[i]=0;knum=0; //ÍË¸ñ
             Disptu(3,White,0,0);
             return;}
 if(k==0x13){Esc();return;}//ÍË³ö
 if(k<=1){if(k==1)Disphzs(0xbfaa,407,curh,White,Black);//¿ª
          else    Disphzs(0xb9d8,407,curh,White,Black);//¹Ø
	        Dispzf24('_',407,curh+76,Magenta,Black);
          knum++;
          if(knum==3){printer=shu[0];
                      bdjc=shu[1];
		                  sgjc=shu[2];
					            SPI_Flash_Read(buf,0x91000,32);	  
					            buf[0]=shu[0];buf[1]=shu[1];buf[2]=shu[2];
					            SPI_Flash_Erase_Sector(145);
                      SPI_Flash_Write_Page(buf,0x91000,32);         
                      Esc();}
         }
}



void EquSetup(void)
{u8 i,k,buf[32];u16 curh;
 if(knum==0)curh=428;
 if(knum==1)curh=504;
 k=shu[knum];
 if(k==0x0d){for(i=0;i<12;i++)shu[i]=0;knum=0; //ÍË¸ñ
             Disptu(3,White,0,0);
             return;}
 if(k==0x13){Esc();return;}//ÍË³ö
 if(k<=1){Dispsz24(k,1,407,curh,Magenta,Black);
	        Dispzf24('_',407,curh+76,Magenta,Black);
          knum++;
          if(knum==2){CRTxj =shu[0];
		                  zxztb[0]=shu[1];
					            SPI_Flash_Read(buf,0x91000,32);  
					            buf[16]=shu[0];buf[17]=shu[1];
					            SPI_Flash_Erase_Sector(145);
                      SPI_Flash_Write_Page(buf,0x91000,32);         
                      Esc();}
         }
}




void Area(void)
{u8 i,k=0,buf[32];
 u16 curh;
 curh=428+16*knum;
 k=shu[knum];
 if(k==0x0d)
     {for(i=0;i<10;i++)shu[i]=0;knum=0; //ÍË¸ñ
      Disptu(4,White,0,0);
      return;}
 if(k==0x13){Esc();return;}//ÍË³ö
 Dispsz24(k,1,407,curh,White,Black);Dispzf24('_',407,curh+16,White,Black);knum++;
 if(knum==3)
    {dstr=DJMenu;Dispstr(10,2,407,500,White,Black);
     machine=shu[0]*100+shu[1]*10+shu[2];
	   SPI_Flash_Read(buf,0x91000,32);	
     buf[4]=machine;			         
     SPI_Flash_Erase_Sector(145);  					
     SPI_Flash_Write_Page(buf,0x91000,32);
     for(i=0;i<10;i++)shu[i]=0;knum=0;MenuN=0;
     return;}
 }
 
 
 
void Logon(void)
{u8 i,k=0,buf[15];u16 num=0,curh;
 curh=500+16*knum;k=shu[knum];
 if(k==0x0d)
  loop1:{for(i=0;i<10;i++)shu[i]=0;knum=0; //ÍË¸ñ
		     Disptu(5,White,0,0);
         return;}
 if(k==0x13){Esc();return;}//ÍË³ö
 Dispzf24(0x2a,407,curh,White,Black);
 Dispzf24('_',407,curh+16,White,Black);knum++;
 if(knum==4)
    {num=1000*shu[0]+100*shu[1]+10*shu[2]+shu[3];
		 if(num==2310){password=0xff;goto loop1;}
     if(password==0xff){if(passf>=num){password=num;goto loop1;}
                        else goto loop;
                       } 
     if(password>0&&password<=3)
{pass[password-1]=num;		                                
 SPI_Flash_Read(buf,0x90000,6);
 buf[0]=pass[0]>>8;buf[1]=pass[0]&0x00ff;buf[2]=pass[1]>>8;buf[3]=pass[1]&0x00ff;
 buf[4]=pass[2]>>8;buf[5]=pass[2]&0x00ff;
 SPI_Flash_Erase_Sector(144);
 SPI_Flash_Write_Page(buf,0x90000,6);
 Dispsz24(password,1,407,672,White,Black);Disphzs24(0xbcb6,407,688,White,Black);dstr=DLMenu;Dispstr(5,4,407,704,White,Black);
}		 
//Õý³£ÃÜÂëµÇÂ¼
	   if(num==pass[0]){passf=1;dstr=DLMenu;Dispstr(9,2,407,564,White,Black);}
     if(num==pass[1]){passf=2;dstr=DLMenu;Dispstr(9,2,407,564,White,Black);}
     if(num==pass[2]){passf=3;dstr=DLMenu;Dispstr(9,2,407,564,White,Black);}
     if(num==4444){passf=0;dstr=DLMenu;Dispstr(13,2,407,564,White,Black);}//ÍË³ö
	   if(num==9876){passf=4;dstr=DLMenu;Dispstr(9,2,407,564,White,Black);}
     if(num==2563){dstr=DLMenu;Dispstr(9,2,407,564,White,Black);//»Ö¸´³ö³§ÉèÖÃ
			             SPI_Flash_Read(buf,0x90000,6);
					         buf[0]=0xff;buf[1]=0xff;buf[2]=0xff;buf[3]=0xff;buf[4]=0xff;buf[5]=0xff;
				           SPI_Flash_Erase_Sector(144);
                   SPI_Flash_Write_Page(buf,0x90000,6);
                  }
     if(num==5380){if(counter>=3200)Set_point();} 
 loop:for(i=0;i<12;i++)shu[i]=0;knum=0;MenuN=0;password=0;return;}
 } 

 


void Init_RxMes(CanRxMsg *RxMessage)
{
  u8 i;
	/*°Ñ½ÓÊÕ½á¹¹ÌåÇåÁã*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_Id_Standard ;    	//±ê×¼±êÊ¶·û           
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0; i < 8; i++){RxMessage->Data[i] = 0x00;}
}

	 
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
//	u8 i = 0;
					 
  TxMessage->ExtId=0x1314;					 
  TxMessage->IDE=CAN_Id_Extended;		          
  TxMessage->RTR=CAN_RTR_Data;			
  TxMessage->DLC=8;							    
	/*ÉèÖÃÒª·¢ËÍµÄÊý¾Ý0-7*/
	//for (i = 0; i< 8; i++){TxMessage->Data[i] = i;}
	TxMessage->Data[0]=0x33;//
  TxMessage->Data[1]=0xFE;// 
}	 
	 
	  

void DMA_transfer(void)
{
 u8 i;

 for (i=0;i<12;i++) { strbup[wtsumup][i] = wtsumup;}	
 USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
 wtsumup++;if(wtsumup==32) wtsumup=0;
} 


void BKP_Read(void)
{
 ctime[4]= RTC_ReadBackupRegister(RTC_BKP_DR2);
 ctime[3]= RTC_ReadBackupRegister(RTC_BKP_DR3);
 ctime[2]= RTC_ReadBackupRegister(RTC_BKP_DR4);
 ctime[1]= RTC_ReadBackupRegister(RTC_BKP_DR5);
 ctime[0]= RTC_ReadBackupRegister(RTC_BKP_DR6);
}


void BKP_Write(void)
{
 RTC_WriteBackupRegister(RTC_BKP_DR2, ctime[4]); 
 RTC_WriteBackupRegister(RTC_BKP_DR3, ctime[3]); 
 RTC_WriteBackupRegister(RTC_BKP_DR4, ctime[2]); 
 RTC_WriteBackupRegister(RTC_BKP_DR5, ctime[1]); 
 RTC_WriteBackupRegister(RTC_BKP_DR6, ctime[0]); 
 //PWR_BackupAccessCmd(DISABLE); //½ûÖ¹Ð´Èë±¸·ÝÓò
}


void RTC_Time_Init(void)
{
  RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;

 // ³õÊ¼»¯Ê±¼ä
	
 RTC_DateStructure.RTC_WeekDay = 1;
 RTC_DateStructure.RTC_Date = 1;
 RTC_DateStructure.RTC_Month = 1;
 RTC_DateStructure.RTC_Year = 23;
 RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
		
 RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
 RTC_TimeStructure.RTC_Hours = 1;
 RTC_TimeStructure.RTC_Minutes = 1;
 RTC_TimeStructure.RTC_Seconds = 0;
 RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
	
 RTC_WriteBackupRegister(RTC_BKP_DR1, 0xa5a5);
}

 

void Get_time(void)
{ // »ñÈ¡ÈÕÀú
	u8 j;
//  RTC_TimeTypeDef RTC_TimeStructure;
//  RTC_DateTypeDef RTC_DateStructure;
//  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
//  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
	j=Ds1302Read(0x8d);	
	ctime[4]=j;	//RTC_DateStructure.RTC_Year;
	j=Ds1302Read(0x89); if(j>=16) j=j-6;
	ctime[3]=j;		//RTC_DateStructure.RTC_Month;
	j=Ds1302Read(0x87); j=(j>>4)*10 +(j &0x0f);
	ctime[2]=j;		//RTC_DateStructure.RTC_Date;
	j=Ds1302Read(0x85); j=((j&0x30)>>4)*10 +(j &0x0f);
	ctime[1]=j;		//RTC_TimeStructure.RTC_Hours;
	j=Ds1302Read(0x83); j=(j>>4)*10 +(j &0x0f);
	ctime[0]=j;		//RTC_TimeStructure.RTC_Minutes;
 
	j=Ds1302Read(0x81); //sec
/*  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;
  // »ñÈ¡ÈÕÀú
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		
	ctime[1]=RTC_TimeStructure.RTC_Hours;
	ctime[0]=RTC_TimeStructure.RTC_Minutes;
 
	ctime[4]=RTC_DateStructure.RTC_Year;
	ctime[3]=RTC_DateStructure.RTC_Month;
	ctime[2]=RTC_DateStructure.RTC_Date;
	Tfile[4]=ctime[4];Tfile[3]=ctime[3];Tfile[2]=ctime[2];Tfile[1]=ctime[1];Tfile[0]=ctime[0];
*/
}


 void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
 {
 /* ·¢ËÍÒ»¸ö×Ö½ÚÊý¾Ýµ½ USART */
 USART_SendData(pUSARTx,ch);
 while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
 }
 
 void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
 uint k=0;
 do {Usart_SendByte( pUSARTx, *(str + k) );
     k++;
    } while (*(str + k)!='\0');
 while (USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET) {}
 }

 
 

void Disptu(u8 numb,u16 color,u16 curv,u16 curh) //FlashÃ¿Ò³´æ1·ùÍ¼
{u8 i;
 switch(numb)
 {case 0://Ö÷½çÃæ
		ClearS(0,0,800,480,Blue);						// µ×É«
		LCD_DrawSquareBox(left_x-1,0,righ_x,25,Magenta);						// ÉÏ·ù	
		LCD_DrawSquareBox(left_x-1,HJtop_y,righ_x,GZlow_y,Black);		// all
		LCD_DrawSquare(left_x-1,HJtop_y-1,righ_x,HJlow_y,Red);				// »ð¾¯1
		LCD_DrawSquare(left_x-1,HJtop_y-1+27,righ_x,HJlow_y-27,Red);	// »ð¾¯1
		LCD_DrawSquare(left_x-1,HJtop_y-1+54,righ_x,HJlow_y-54,Red);	// »ð¾¯1
		LCD_DrawSquare(left_x-1,HJtop_y-1+81,righ_x,HJlow_y-81,Red);	// »ð¾¯1
		LCD_DrawSquare(left_x-1,HJtop_y-1+108,righ_x,HJlow_y-108,Red);// »ð¾¯1
		LCD_DrawSquare(left_x-1,LDtop_y-1,righ_x,LDlow_y,Cyan);				// Áª¶¯
		LCD_DrawSquare(left_x-1,LDtop_y-1+27,righ_x,LDlow_y,Cyan);	// Áª¶¯
		LCD_DrawSquare(left_x-1,LDtop_y-1+54,righ_x,LDlow_y,Cyan);	// Áª¶¯
		LCD_DrawSquare(left_x-1,LDtop_y-1+81,righ_x,LDlow_y,Cyan);	// Áª¶¯
		LCD_DrawSquare(left_x-1,LDtop_y-1+108,righ_x,LDlow_y,Cyan);	// Áª¶¯
		LCD_DrawSquare(left_x-1,GZtop_y-1,righ_x,GZlow_y,Green);			// ¹ÊÕÏ
		LCD_DrawSquare(left_x-1,GZtop_y-1+27,righ_x,GZlow_y,Green);			// ¹ÊÕÏ
		LCD_DrawSquare(left_x-1,GZtop_y-1+54,righ_x,GZlow_y,Green);			// ¹ÊÕÏ
		LCD_DrawSquare(left_x-1,GZtop_y-1+81,righ_x,GZlow_y,Green);			// ¹ÊÕÏ
		LCD_DrawSquare(left_x-1,GZtop_y-1+108,righ_x,GZlow_y,Green);			// ¹ÊÕÏ

		LCD_DrawSquare(left_x-1,LDtop_y-3,righ_x,2,White);			// »ð¾¯Áª¶¯ line
		LCD_DrawSquare(left_x-1,GZtop_y-3,righ_x,2,White);			// Áª¶¯¹ÊÕÏ line
		LCD_DrawSquare(left_x-1,GZtop_y-2+108,righ_x,2,White);			// ¹ÊÕÏPB line

//		LCD_DrawSquareBox(left_y,408,righ_y,24,Magenta);			//PingBi
//±¨¾¯Í·
/*
//u16 nob_x=26,alm_x=26+33,nam_x=26+33+49,zon_x=26+33+49+98;
//u16 flo_x=26+33+49+98+74,rom_x=26+33+49+98+74+28;
//u16 tim_x=26+33+49+98+74+28+194;     //26, 33, 49, 74, 28, 194,112
//u16 adr_x=26+33+49+98+74+28+194+112; //tou nob alm zon flo rom adr

		
		
		
		
		
		
		dstr=HMenu;Dispstr(32,2,0,nob_x,Magenta,Green);//ÐòºÅ
		dstr=HMenu;Dispstr(0,2,0,alm_x+12,Magenta,Green);//±¨¾¯
		dstr=HMenu;Dispstr(8,4,0,nam_x+12,Magenta,Green);//Éè±¸Ãû³Æ
		dstr=HMenu;Dispstr(16,2,0,zon_x+12,Magenta,Green);//·ÖÇ
		dstr=HMenu;Dispstr(20,2,0,flo_x-6,Magenta,Green);//Â¥²ãø	
		dstr=HMenu;Dispstr(24,2,0,rom_x+48,Magenta,Green);//×¢ÊÍ
		dstr=HMenu;Dispstr(28,2,0,tim_x+48,Magenta,Green);//Ê±¼ä
		dstr=HMenu;Dispstr(4,2,0,adr_x+12,Magenta,Green);//µØÖ·
*/
		dstr=HMenu;Dispstr(0,2,0,alm_x+12,Magenta,Green);//±¨¾¯
		dstr=HMenu;Dispstr(4,2,0,adr_x+12,Magenta,Green);//µØÖ·
		dstr=HMenu;Dispstr(8,4,0,nam_x+12,Magenta,Green);//Éè±¸Ãû³Æ
		dstr=HMenu;Dispstr(16,2,0,zon_x+12,Magenta,Green);//·ÖÇø	
		dstr=HMenu;Dispstr(20,2,0,flo_x-6,Magenta,Green);//Â¥²ã
		dstr=HMenu;Dispstr(24,2,0,rom_x+48,Magenta,Green);//×¢ÊÍ
		dstr=HMenu;Dispstr(28,2,0,tim_x+48,Magenta,Green);//Ê±¼ä
		dstr=HMenu;Dispstr(32,2,0,nob_x,Magenta,Green);//ÐòºÅ

		Disphzs24(0xcad7,25,0,Blue,White);		//shou
		Disphzs24(0xbbf0,84,0,Blue,White);		//»ð
		Disphzs24(0xbeaf,120,0,Blue,White);	  //¾¯ 
		Disphzs24(0xc1aa,204,0,Blue,White);	  //Áª
		Disphzs24(0xb6af,240,0,Blue,White);	  //¶¯ 
		Disphzs24(0xb9ca,324,0,Blue,White);	  //¹Ê
		Disphzs24(0xd5cf,348,0,Blue,White);	  //ÕÏ
		Disphzs(0xc6c1,400,0,Blue,White);	  //Ping
		Disphzs(0xb1ce,418,0,Blue,White);	  //Bi

		Dispsys();
		DispFactor();
		DispTimeUP();	 
		break;
		
		case 1://±¨¾¯Í·
			      ClearS(0,0,800,24,Magenta);
			      dstr=HMenu;Dispstr(0,2,0,0,Magenta,White);
            dstr=HMenu;Dispstr(4,2,0,56,Magenta,White);
            dstr=HMenu;Dispstr(8,4,0,112,Magenta,White);
            dstr=HMenu;Dispstr(16,2,0,256,Magenta,White);	
            dstr=HMenu;Dispstr(20,2,0,312,Magenta,White);
            dstr=HMenu;Dispstr(24,2,0,368,Magenta,White);
            dstr=HMenu;Dispstr(28,2,0,632,Magenta,White);
            dstr=HMenu;Dispstr(32,2,0,744,Magenta,White);
            break;
		case 2://Ö÷½çÃæÅ¨¶ÈÖµ
            ClearS(404,0,800,75,White);ClearS(404,0,800,25,Cyan);
		        LCD_DrawLine(0,404,800,0,Blue);LCD_DrawLine(0,429,800,0,Blue);LCD_DrawLine(0,454,800,0,Blue);//µÚ1,2,3ÐÐºáÏß 
		        Disphzs24(0xb5d8,430,0,Cyan,Blue);Disphzs24(0xd6b7,430,24,Cyan,Blue);
	          Disphzs24(0xc5a8,455,0,Cyan,Blue);Disphzs24(0xb6c8,455,24,Cyan,Blue);
	          LCD_DrawLine(48,430,50,1,Blue);LCD_DrawLine(118,430,50,1,Blue);LCD_DrawLine(186,430,50,1,Blue);//µÚ1,2,3ÁÐÊúÏß
		        LCD_DrawLine(254,430,50,1,Blue);LCD_DrawLine(322,430,50,1,Blue); LCD_DrawLine(390,430,50,1,Blue);//µÚ4,5,6ÁÐÊúÏß
		        LCD_DrawLine(458,430,50,1,Blue);LCD_DrawLine(526,430,50,1,Blue);LCD_DrawLine(594,405,75,1,Blue);//µÚ7,8,9ÁÐÊúÏß					
            break;
		case 3://ÏµÍ³²ÎÊý
			      ClearXY(407,380,24,420,White);
			      Disphzs24(0xb4f2,407,380,White,Black);Disphzs24(0xd3a1,407,404,White,Black);
            Disphzs24(0xb1b8,407,452,White,Black);Disphzs24(0xb5e7,407,476,White,Black);
            Disphzs24(0xc9f9,407,532,White,Black);Disphzs24(0xb9e2,407,556,White,Black);
            //Dispzf24('C',407,604,White,Black);Dispzf24('R',407,620,White,Black);Dispzf24('T',407,634,White,Black);	
            Dispzf24('_',407,428,White,Black);		 
			      break;
		case 4://ÁªÍø
			      ClearXY(407,380,24,420,White);
		        dstr=HZMenuF2;Dispstr(6,2,407,380,White,Black);
            Dispzf24('_',407,428,White,Black);		 
			      break;
		case 5://ÃÜÂë
			      ClearXY(407,380,24,420,White);
		        dstr=DLMenu;Dispstr(0,5,407,380,White,Black);
            Dispzf24('_',407,500,White,Black);		 
			      break;
		case 6://Íâ²¿Ñ¡ÅäÉè±¸
			      ClearXY(407,380,24,420,White);
			      Dispzf24('C',407,380,White,Black);Dispzf24('R',407,396,White,Black);Dispzf24('T',407,412,White,Black);
            Disphzs24(0xb6E0,407,452,White,Black);Disphzs24(0xcfdf,407,476,White,Black);
		        //Disphzs24(0xb2e3,407,532,White,Black);Disphzs24(0xcfd4,407,556,White,Black);
            Dispzf24('_',407,428,White,Black);		 
			      break;
		case 14://µãÎ»ÉèÖÃ
            ClearS(0,0,800,480,Magenta);
            Disphzs24(0xb5e3,64,64,Magenta,Blue);Disphzs24(0xcebb,64,88,Magenta,Blue);
for(i=1;i<8;i++)
{Dispsz24(i,1,100+i*24,64,Magenta,White);
 Dispsz24(i,1,100+i*24,112,Magenta,Black);Disphzs24(0xbbd8,100+i*24,128,Magenta,Black);Disphzs24(0xc2b7,100+i*24,152,Magenta,Black);//»ØÂ·
 Dispsz24(64*i,3,100+i*24,224,Magenta,Black);Disphzs24(0xb5e3,100+i*24,272,Magenta,Black);}
            break;
		case 15://³õÊ¼»¯½çÃæ
            LCD_SetLayer(LCD_BACKGROUND_LAYER);//°Ñ±³¾°²ãË¢ºÚÉ«
            LCD_SetLayer(LCD_FOREGROUND_LAYER);//Ç°¾°²ã 
            LCD_SetTransparency(0xff);//²»Í¸Ã÷  
			      ClearS(0,0,800,480,Magenta);
            Disphzs24(0xb3f5,50,388,Magenta,Black);Disphzs24(0xcabc,50,412,Magenta,Black);Disphzs24(0xbbaf,50,436,Magenta,Black);//³õÊ¼»¯
            break;
		case 21://²Ëµ¥
		         ClearXY(455,380,24,420,DarkGreen);
             dstr=HZMenu;Dispstr(0,9,455,400,DarkGrey,White);//1ÉèÖÃ2²Ù×÷3µµ°¸
             break;	 
		case 22://¡°ÉèÖÃ¡±×Ó²Ëµ¥
		         dstr=HZMenu;Dispstr(0,3,455,400,DarkGrey,Blue);
		         ClearXY(431,380,24,420,DarkGrey);
             dstr=HZMenuF1;Dispstr(0,15,431,400,DarkGrey,Black);//4Ê±ÖÓ5µÇÂ¼6µÇ¼Ç7ÊÖµÇ8Éè±¸
		         break;	 
		case 23://¡°²Ù×÷¡±×Ó²Ëµ¥
		         dstr=HZMenu;Dispstr(0,9,455,400,DarkGrey,White);dstr=HZMenu;Dispstr(5,3,455,472,DarkGrey,Blue);//2²Ù×÷
		         ClearXY(431,380,24,420,DarkGrey);
		         dstr=HZMenuF2;Dispstr(0,9,431,400,DarkGrey,Black);//4Æô¶¯5ÁªÍø6¼ì²â
		         break; 
		case 24://¡°µµ°¸¡±×Ó²Ëµ¥
		         dstr=HZMenu;Dispstr(0,9,455,400,DarkGrey,White);dstr=HZMenu;Dispstr(10,3,455,544,DarkGrey,Blue);//3µµ°¸
		         ClearXY(431,380,24,420,DarkGrey);
		         dstr=HZMenuF3;Dispstr(0,9,431,400,DarkGrey,Black);//4µÇ¼Ç5ÆÁ±Î6¹ÊÕÏ
		         break;
		case 25://±à³Ì²Ëµ¥
	           ClearS(0,0,800,480,White);
	           dstr=XCMenu;Dispstr(0,5,48,48,White,Black);
		         dstr=XCMenu;Dispstr(9,5,96,48,White,Black);
		         dstr=XCMenu;Dispstr(18,5,144,48,White,Black);
		         dstr=XCMenu;Dispstr(27,5,192,48,White,Black);
		         dstr=XCMenu;Dispstr(36,5,240,48,White,Black);
		         dstr=XCMenu;Dispstr(45,5,288,48,White,Black);
		         dstr=XCMenu;Dispstr(54,5,336,48,White,Black);
		         break;
		case 26://ÎÄ¼þÒÑ±£´æ	 
		         dstr=FLMenu;Dispstr(0,5,curv,curh,Magenta,Yellow);
             break;	
		case 27:LCD_DrawSquare(curh,curv,160,16,color);ClearXY(curv,curh,15,79,color); 
             break;
		case 28:ClearXY(curv,curh,15,79,color);
		         break;
		case 29://ÎÄ¼þ	 
			       ClearS(0,0,800,480,Magenta);//ÇåÆÁ
             Disphzs24(0xbbd8,30,48,Magenta,Blue);Disphzs24(0xc2b7,30,72,Magenta,Blue);Dispzf24('_',30,120,Magenta,Blue);//»ØÂ·
		         LCD_DrawSquare(190,28,30,30,Black);Dispzf24(0x1a,30,200,Magenta,Black);//¡ú
             Dispzf24(0x1a,30,224,Magenta,Blue);Disphzs24(0xb8b4,30,248,Magenta,Blue);Disphzs24(0xd6c6,30,272,Magenta,Blue);//¸´ÖÆ
             LCD_DrawSquare(340,28,30,30,Black);Dispzf24(0x18,30,350,Magenta,Black);//¡ý
             Dispzf24(0x1a,30,374,Magenta,Black);Disphzs24(0xc8b7,30,398,Magenta,Black);Disphzs24(0xb6a8,30,422,Magenta,Black);//È·¶¨      
             LCD_DrawSquare(490,28,60,30,Black);Disphzs24(0xb1e0,30,500,Magenta,Black);Disphzs24(0xb3cc,30,524,Magenta,Black);//±à³Ì
             Dispzf24(0x1a,30,558,Magenta,Black);Disphzs24(0xb1a3,30,582,Magenta,Black);Disphzs24(0xb4e6,30,606,Magenta,Black);//±£´æ
             break;	
		case 32://ÏÔÊ¾±à³Ì
		        dstr=XCBCMenu;Dispstr(0,12,48,216,White,Blue);	
		        break;
		case 33://Áª¶¯±à³Ì	    
	        	dstr=XCBCMenu;Dispstr(0,12,96,216,White,Blue);	
	          break;
     case 34://Éè±¸Ãû³Æ	    
		        dstr=XCBCMenu;Dispstr(0,12,144,216,White,Blue);		
		        break;
     case 35://²ãÏÔ±à³Ì   
            dstr=XCCXMenu;Dispstr(0,12,192,216,White,Blue);					 
		        break;
		 case 38://×ÜÏß±à³Ì   
            dstr=XCCXMenu;Dispstr(0,12,240,216,White,Blue);	 
		        break; 
     case 36://Õû»ú²âÊÔ	    
		        dstr=XCCSMenu;Dispstr(0,12,288,216,White,Blue);	
		        break;
     case 37://ÏµÍ³¼ì²é	    
		        dstr=XCXTMenu;Dispstr(0,7,336,216,White,Blue);
	          break;
		 case 40://ÏÔÊ¾±à³Ì	
			      ClearS(64,0,800,416,Magenta);
            dstr=XSMenu;Dispstr(0,15,144,0,Magenta,Black);
            break;
		 case 42://Éè±¸Ãû³Æ
             ClearS(0,0,800,480,Magenta);
             dstr=XSMenu;Dispstr(25,2,144,0,Magenta,Black);
             dstr=XCMenu;Dispstr(19,4,144,96,Magenta,White);
             break;	 
		 case 43://Áª¶¯±à³Ì	
             ClearS(0,0,800,480,Magenta);
             Disphzs24(0xb9d8,48,0,Magenta,Black);Disphzs24(0xcfb5,48,24,Magenta,Black);  
             Disphzs24(0xccf5,96,0,Magenta,Black);Disphzs24(0xbcfe,96,24,Magenta,Black);  
             Disphzs24(0xc1aa,168,0,Magenta,Black);Disphzs24(0xb6af,168,24,Magenta,Black);
             break;
     case 44://±¨¾¯ÏÞ			 
		         Dispalm();
Disphzs24(0xbbd8,0,16,Magenta,Black);Disphzs24(0xc2b7,0,40,Magenta,Black);//»ØÂ·
Disphzs24(0xb1a8,0,100,Magenta,Black);Disphzs24(0xbeaf,0,124,Magenta,Black);Disphzs24(0xd6b5,0,148,Magenta,Black);//±¨¾¯ÖµÉèÖÃ
Disphzs24(0xc9e8,0,172,Magenta,Black);Disphzs24(0xd6c3,0,196,Magenta,Black); 
//Dispzf24(27,0,400,Magenta,Blue);Disphzs24(0xb1e0,0,416,Magenta,White);Disphzs24(0xb3cc,0,440,Magenta,White); //¡û±à³Ì
//Dispzf24(26,0,500,Magenta,Red);Disphzs24(0xc7e5,0,516,Magenta,White);Disphzs24(0xbfd5,0,540,Magenta,White); //¡úÇå¿Õ
//Dispzf24(0x26,0,600,Magenta,White);Disphzs24(0xc9cf,0,616,Magenta,White);Disphzs24(0xb4ab,0,640,Magenta,White); //&ÉÏ´«
		         break;
		 case 45://²¹³¥ÏÞ
			       Dispalm();
Disphzs24(0xbbd8,0,16,Magenta,Black);Disphzs24(0xc2b7,0,40,Magenta,Black);//»ØÂ·
Disphzs24(0xb2b9,0,100,Magenta,Black);Disphzs24(0xb3a5,0,124,Magenta,Black);Disphzs24(0xc9e8,0,148,Magenta,Black);Disphzs24(0xd6c3,0,172,Magenta,Black);//²¹³¥ÉèÖÃ    
Dispzf24(27,0,400,Magenta,Blue);Disphzs24(0xb1e0,0,416,Magenta,White);Disphzs24(0xb3cc,0,440,Magenta,White); //¡û±à³Ì
Dispzf24(26,0,500,Magenta,Red);Disphzs24(0xc7e5,0,516,Magenta,White);Disphzs24(0xbfd5,0,540,Magenta,White); //¡úÇå¿Õ
Dispzf24(0x26,0,600,Magenta,White);Disphzs24(0xc9cf,0,616,Magenta,White);Disphzs24(0xb4ab,0,640,Magenta,White); //&ÉÏ´«
		         break;	 
		 case 46://±¨¾¯¡¢²¹³¥ÉèÖÃ
			       ClearXY(0,300,48,500,Magenta);
             Disphzs24(0xb5d8,0,350,Magenta,Red);Disphzs24(0xd6b7,0,374,Magenta,Red);//µØÖ·
             Disphzs24(0xb5cd,0,500,Magenta,Blue);Disphzs24(0xcfde,0,524,Magenta,Blue);//µÍÏÞ
             Disphzs24(0xb8df,0,650,Magenta,Blue);Disphzs24(0xcfde,0,674,Magenta,Blue);//¸ßÏÞ
		         break;
		 case 47://²ÎÊý²éÑ¯
			       //ClearS(0,0,800,480,Magenta);
			       ClearXY(0,200,24,600,Magenta);
             Dispzf24(26,0,200,Magenta,Blue);Disphzs24(0xc3fc,0,216,Magenta,White);Disphzs24(0xc1ee,0,240,Magenta,White);//¡úÃüÁî
             Disphzs24(0xc3dc,0,400,Magenta,White);Disphzs24(0xc2eb,0,424,Magenta,White);//ÃÜÂë
		         break;
		 case 48://Í¨Ñ¶²âÊÔ
             ClearS(0,0,800,480,White);
		         ClearXY(0,0,48,800,Magenta);	
dstr=XCCSMenu;Dispstr(11,2,0,0,Magenta,Blue);//Í¨Ñ¶
Disphzs24(0xb2a8,0,112,Magenta,Blue);Disphzs24(0xccd8,0,136,Magenta,Blue);Disphzs24(0xc2ca,0,160,Magenta,Blue);//²¨ÌØÂÊ
Disphzs24(0xd0a3,0,288,Magenta,Blue);Disphzs24(0xd1e9,0,312,Magenta,Blue);Disphzs24(0xcebb,0,336,Magenta,Blue);//Ð£ÑéÎ»
Dispzf24('N',0,360,Magenta,Black);Dispzf24('O',0,376,Magenta,Black);Dispzf24('N',0,392,Magenta,Black);Dispzf24('E',0,408,Magenta,Black);
Disphzs24(0xcafd,0,488,Magenta,Blue);Disphzs24(0xbedd,0,512,Magenta,Blue);Disphzs24(0xcebb,0,536,Magenta,Blue);//Êý¾ÝÎ»
Dispsz24(8,1,0,560,Magenta,Black);
Disphzs24(0xcda3,0,600,Magenta,Blue);Disphzs24(0xd6b9,0,624,Magenta,Blue);Disphzs24(0xcebb,0,648,Magenta,Blue);//Í£Ö¹Î»
Dispsz24(1,1,0,672,Magenta,Black);		
             break;
		 case 49://´òÓ¡¼ÇÂ¼
             ClearS(0,0,800,480,Magenta);	
Disphzs24(0xbcc7,100,64,Magenta,White);Disphzs24(0xc2bc,100,88,Magenta,White);//¼ÇÂ¼
Dispzf24('1',100,200,Magenta,Blue);Disphzs24(0xbbf0,100,216,Magenta,Blue);Disphzs24(0xbeaf,100,240,Magenta,Blue);//1»ð¾¯
Dispzf24('3',100,350,Magenta,Blue);Disphzs24(0xc6e4,100,366,Magenta,Blue);Disphzs24(0xcbfc,100,390,Magenta,Blue);//3ÆäËü
Disphzs24(0xb7bd,148,64,Magenta,White);Disphzs24(0xcabd,148,88,Magenta,White);//·½Ê½	 
Dispzf24('1',148,200,Magenta,Blue);Disphzs24(0xd6b8,148,216,Magenta,Blue);Disphzs24(0xb6a8,148,240,Magenta,Blue);//1Ö¸¶¨ÈÕÆÚ
Disphzs24(0xc8d5,148,264,Magenta,Blue);Disphzs24(0xc6da,148,288,Magenta,Blue);
Dispzf24('2',148,350,Magenta,Blue);Disphzs24(0xd6b8,148,366,Magenta,Blue);Disphzs24(0xb6a8,148,390,Magenta,Blue);//2Ö¸¶¨ÐòºÅ
Disphzs24(0xd0f2,148,414,Magenta,Blue);Disphzs24(0xbac5,148,438,Magenta,Blue);
             break;
		 case 50://´òÓ¡¼ÇÂ¼¡ª¡ª°´ÈÕÆÚ	
			       ClearXY(200,0,24,800,Magenta);	
             dstr=TMenu;Dispstr(0,5,200,64,Magenta,Blue);//Äê/ÔÂ/ÈÕ
             break;
		 case 51://´òÓ¡¼ÇÂ¼¡ª¡ª°´ÐòºÅ
             ClearXY(200,0,24,800,Magenta);					
Disphzs24(0xd0f2,200,64,Magenta,Blue);Disphzs24(0xbac5,200,88,Magenta,Blue);Dispzf24('-',200,200,Magenta,Blue);//ÐòºÅ		
             break;
		 case 52://ÕýÔÚ´òÓ¡
              Disphzs24(0xd5fd,400,64,Magenta,Black);Disphzs24(0xd4da,400,88,Magenta,Black);
              Disphzs24(0xb4f2,400,112,Magenta,Black);Disphzs24(0xd3a1,400,136,Magenta,Black);
              break;
		 case 53://²ãÏÔ±à³Ì	
             ClearS(0,0,800,480,Magenta);	 
		         Disphzs24(0xbbf0,48,120,Magenta,Red);Disphzs24(0xbeaf,48,144,Magenta,Red);//»ð¾¯
		         Disphzs24(0xb2e3,48,264,Magenta,Blue);Disphzs24(0xcfd4,48,288,Magenta,Blue);Dispzf24('1',48,312,Magenta,Blue);//²ãÏÔ1
		         Disphzs24(0xb2e3,48,408,Magenta,Blue);Disphzs24(0xcfd4,48,432,Magenta,Blue);Dispzf24('2',48,456,Magenta,Blue);//²ãÏÔ2		       
             Disphzs24(0xbbd8,72,120,Magenta,Black);Disphzs24(0xc2b7,72,144,Magenta,Black);//»ØÂ·
		         Disphzs24(0xb5d8,72,192,Magenta,Black);Disphzs24(0xd6b7,72,216,Magenta,Black);//µØÖ·		 
		         Disphzs24(0xbbd8,72,264,Magenta,Black);Disphzs24(0xc2b7,72,288,Magenta,Black);//»ØÂ·
		         Disphzs24(0xb5d8,72,336,Magenta,Black);Disphzs24(0xd6b7,72,360,Magenta,Black);//µØÖ·		 
		         Disphzs24(0xbbd8,72,408,Magenta,Black);Disphzs24(0xc2b7,72,432,Magenta,Black);//»ØÂ·
		         Disphzs24(0xb5d8,72,480,Magenta,Black);Disphzs24(0xd6b7,72,504,Magenta,Black);//µØÖ·		 
             break;
		 case 54://×ÜÏßÅÌ±à³Ì	
             ClearS(0,0,800,480,Magenta);
             Disphzs24(0xc4a3,48,120,Magenta,Red);Disphzs24(0xbfe9,48,144,Magenta,Red);//Ä£¿é
		         Disphzs24(0xd7dc,48,264,Magenta,Blue);Disphzs24(0xcfdf,48,288,Magenta,Blue);Disphzs24(0xc5cc,48,312,Magenta,Blue);//×ÜÏßÅÌ    
             Disphzs24(0xbbd8,72,120,Magenta,Black);Disphzs24(0xc2b7,72,144,Magenta,Black);//»ØÂ·
		         Disphzs24(0xb5d8,72,192,Magenta,Black);Disphzs24(0xd6b7,72,216,Magenta,Black);//µØÖ·
		         Disphzs24(0xc5cc,72,264,Magenta,Black);Disphzs24(0xbac5,72,288,Magenta,Black);//ÅÌºÅ
		         Disphzs24(0xb0b4,72,336,Magenta,Black);Disphzs24(0xbcfc,72,360,Magenta,Black);//°´¼ü
             break;
		default:break;
		 		
	} //switch(numb) end
}


void Dispsys(void)
{u8 i; 
 u16 j;
	
 Disphzs(0xcfb5,sys_1y,0,Blue,White);Disphzs(0xcdb3,sys_2y,0,Blue,White);//ÏµÍ³
 LCD_DrawSquareBox(sys_1x,sys_1y,sys_3x-sys_1x,16,Magenta);		// ÏµÍ³ÉÏÌõ
 LCD_DrawSquareBox(sys_1x,sys_2y,sys_3x-sys_1x,16,Magenta);		// ÏµÍ³ÏÂÌõ
 dstr=HZsys; Dispstr16(0,35,sys_2y,sys_1x,Magenta,White);    // ÏµÍ³ÏÂÌõ×Ö 
 Dispsz16(ut6_max,2,sys_1y,sys_1x,Magenta,White);             //10£¨°æ±£©¾£©
 if(ut6_jz==0) j=0xbcaf; else j=0xc7f8;
 Disphzs(j,sys_1y,sys_1x+16,Magenta,White);             //0xb5a5µ¥£¬0xbcaf¼¯ 0xc7f8Çø
 i=1;Dispsz16(VZHJ,4,sys_1y,sys_1x+i*40,Magenta,White);   //»ð¾¯
 i=2;Dispsz16(VZQD,4,sys_1y,sys_1x+i*40,Magenta,White);   //Æô¶¯
 i=3;Dispsz16(VZHD,4,sys_1y,sys_1x+i*40,Magenta,White);   //·´À¡
 i=4;Dispsz16(VZDJ,4,sys_1y,sys_1x+i*40,Magenta,White);   //µÇ¼Ç
 i=5;Dispsz16(VZGZ,4,sys_1y,sys_1x+i*40,Magenta,White);   //¹ÊÕÏ
 i=6;Dispsz16(VZPB,4,sys_1y,sys_1x+i*40,Magenta,White);   //ÆÁ±Î
 i=7;Dispsz16(VZSX,4,sys_1y,sys_1x+i*40,Magenta,White);   //ÉÏÏß
 i=8;Dispsz16(VZLP,4,sys_1y,sys_1x+i*40,Magenta,White);   //ÁªÅÌ
 Disphzs(0xbbd8,sys_1y,sys_1x+360+80,Magenta,White);      //»Ø1
 i=1;Dispsz16(i,1,sys_1y,sys_1x+376+80,Magenta,White);	      //1
 for(i=0;i<8;i++)Dispsz16(LP_Num[i],3,sys_1y,sys_2x+i*27,Magenta,White);	//1-8sub
 for(i=0;i<8;i++)Dispsz16(LP_Num[i+8],3,sys_2y,sys_2x+i*27,Magenta,White);	//9-16sub

//----------------------------------¼ì²â±í--------------------------------------------//
 //ClearXY(389,30,34,249,Yellow);
// LCD_DrawLine(30,388,249,0,Black);//µÚÒ»ÐÐºáÏß
// LCD_DrawLine(63,406,216,0,Black);//µÚ¶þÐÐºáÏß
// LCD_DrawLine(30,424,249,0,Black);//µÚÈýÐÐºáÏß
// LCD_DrawLine(29,388,36,1,Black);//µÚÒ»ÁÐÊúÏß	
// Disphzs(0xbcec,389,31,Yellow,Red);Disphzs(0xb2e2,389,47,Yellow,Red);//¼ì²â
// LCD_DrawLine(63,388,36,1,Black);//µÚ¶þÁÐÊúÏß
// Disphzs(0xb4f2,389,66,Yellow,Blue);Disphzs(0xd3a1,389,82,Yellow,Blue);//´òÓ¡»ú
// if(printer==1)Disphzs(0xbfaa,407,66,Yellow,Magenta);else Disphzs(0xb9d8,407,66,Yellow,Black);
// LCD_DrawLine(100,388,36,1,Black);//µÚÈýÁÐÊúÏß
// Disphzs(0xb1b8,389,101,Yellow,Blue);Disphzs(0xb5e7,389,117,Yellow,Blue);//±¸µç
// if(bdjc==1)Disphzs(0xbfaa,407,101,Yellow,Magenta);else Disphzs(0xb9d8,407,101,Yellow,Black);	
// LCD_DrawLine(135,388,36,1,Black);//µÚËÄÁÐÊúÏß
// Disphzs(0xc9f9,389,136,Yellow,Blue);Disphzs(0xb9e2,389,152,Yellow,Blue);//Éù¹â
// if(sgjc==1)Disphzs(0xbfaa,407,136,Yellow,Magenta);else Disphzs(0xb9d8,407,136,Yellow,Black);
// LCD_DrawLine(170,388,36,1,Black);//µÚÎåÁÐÊúÏß
// Dispzf('C',389,171,Yellow,Black);Dispzf('R',389,179,Yellow,Black);Dispzf('T',389,187,Yellow,Black);//CRT
// Dispsz16(CRTxj,2,407,171,Yellow,Blue);//if(CRTxj)Disphzs(0xbfaa,407,171,Yellow,Magenta);else Disphzs(0xb9d8,407,171,Yellow,Black);
// LCD_DrawLine(205,388,36,1,Black);//µÚÁùÁÐÊúÏß
// Disphzs(0xb6e0,389,206,Yellow,Black);Disphzs(0xcfdf,389,222,Yellow,Black);//¶àÏß
// Dispsz16(zxztb[0],2,407,206,Yellow,Blue);
// LCD_DrawLine(240,388,36,1,Black);//µÚÆßÁÐÊúÏß
// //Disphzs(0xb2e3,389,241,Yellow,Black);Disphzs(0xcfd4,389,257,Yellow,Black);//²ãÏÔ
// //Dispsz16(cxztb[0],2,407,241,Yellow,Blue);
// LCD_DrawLine(277,388,36,1,Black);//µÚ°ËÁÐÊúÏß 
////----------------------------------×´Ì¬±í--------------------------------------------// 
// ClearXY(389,278,90,86,DarkRed);
// LCD_DrawLine(278,388,86,0,Black);//µÚÒ»ÐÐºáÏß
// LCD_DrawLine(278,406,86,0,Black);//µÚ¶þÐÐºáÏß
// LCD_DrawLine(278,424,86,0,Black);//µÚÈýÐÐºáÏß
// LCD_DrawLine(278,442,86,0,Black);//µÚËÄÐÐºáÏß
// LCD_DrawLine(278,460,86,0,Black);//µÚÎåÐÐºáÏß
// LCD_DrawLine(278,479,86,0,Black);//µÚÁùÐÐºáÏß
// LCD_DrawLine(277,388,92,1,Black);//µÚÒ»ÁÐÊúÏß	
// LCD_DrawLine(312,388,92,1,Black);//µÚ¶þÁÐÊúÏß
// LCD_DrawLine(363,388,92,1,Black);//µÚÈýÁÐÊúÏß 
// 

// //Disphzs(0xb5e3,389,279,Maroon,White);Disphzs(0xcebb,389,295,Maroon,White);//µãÎ»
// //if(llpp<=8)Dispsz16(llpp*200,4,389,313,Maroon,Blue);else Dispzf('T',389,329,Maroon,White);
// Disphzs(0xbbd8,389,279,DarkRed,White);Disphzs(0xc2b7,389,295,DarkRed,White);//»ØÂ·
// if(llpp<=8)Dispsz16(llpp,1,389,329,DarkRed,Blue);else Dispzf('T',389,329,DarkRed,White);
// 
// Disphzs(0xc8ed,407,279,DarkRed,White);Disphzs(0xbcfe,407,295,DarkRed,White); 
// Dispzf('1',407,321,DarkRed,Blue);Dispzf('.',407,329,DarkRed,Blue);Dispzf('0',407,337,DarkRed,Blue);
// //È¨ÏÞ
// Disphzs(0xc8a8,425,279,DarkRed,White);Disphzs(0xcfde,425,295,DarkRed,White);
// Dispsz16(passf,1,425,329,DarkRed,Blue);
// //·½Ê½
// Disphzs(0xb7bd,443,279,DarkRed,White);Disphzs(0xcabd,443,295,DarkRed,White);
// if(machine==0){Disphzs(0xb6c0,443,313,DarkRed,Cyan);Disphzs(0xc1a2,443,329,DarkRed,Cyan);Disphzs(0xcabd,443,345,DarkRed,Cyan);}
// else{Disphzs(0xc7f8,443,313,DarkRed,Cyan);Disphzs(0xd3f2,443,329,DarkRed,Cyan);Disphzs(0xbbfa,443,345,DarkRed,Cyan);}
// //µØÖ·(ÁªÍø)
// Disphzs(0xb5d8,461,279,DarkRed,White);Disphzs(0xd6b7,461,295,DarkRed,White);
// Dispsz16(machine,3,461,313,DarkRed,Cyan);
////----------------------------------»ØÂ·±í--------------------------------------------// 
// ClearXY(425,30,54,247,Green);
// LCD_DrawLine(30,425,249,0,DarkGreen);//µÚÒ»ÐÐºáÏß
// LCD_DrawLine(30,443,249,0,DarkGreen);//µÚ¶þÐÐºáÏß
// LCD_DrawLine(30,461,249,0,DarkGreen);//µÚÈýÐÐºáÏß
// LCD_DrawLine(30,479,249,0,DarkGreen);//µÚËÄÐÐºáÏß
// LCD_DrawLine(29,425,54,1,DarkGreen);//µÚÒ»ÁÐÊúÏß
// Disphzs(0xbbd8,426,30,Green,Black);Disphzs(0xc2b7,426,46,Green,Black);//»ØÂ·
// Disphzs(0xb5c7,444,30,Green,Black);Disphzs(0xbcc7,444,46,Green,Black);//µÇ¼Ç					
// Disphzs(0xd4da,462,30,Green,Black);Disphzs(0xcfdf,462,46,Green,Black);//ÔÚÏß
// LCD_DrawLine(63,425,54,1,DarkGreen);//µÚ¶þÁÐÊúÏß
// Dispsz16(1,1,426,65,Green,Black);LCD_DrawLine(90,425,54,1,DarkGreen);//µÚÈýÁÐÊúÏß
// Dispsz16(2,1,426,92,Green,Black);LCD_DrawLine(117,425,54,1,DarkGreen);//µÚËÄÁÐÊúÏß
// Dispsz16(3,1,426,119,Green,Black);LCD_DrawLine(144,425,54,1,DarkGreen);//µÚÎåÁÐÊúÏß
// Dispsz16(4,1,426,146,Green,Black);LCD_DrawLine(171,425,54,1,DarkGreen);//µÚÁùÁÐÊúÏß
// Dispsz16(5,1,426,173,Green,Black);LCD_DrawLine(198,425,54,1,DarkGreen);//µÚÆßÁÐÊúÏß
// Dispsz16(6,1,426,200,Green,Black);LCD_DrawLine(225,425,54,1,DarkGreen);//µÚ°ËÁÐÊúÏß
// Dispsz16(7,1,426,227,Green,Black);LCD_DrawLine(252,425,54,1,DarkGreen);//µÚ¾ÅÁÐÊúÏß
// Dispsz16(8,1,426,254,Green,Black);LCD_DrawLine(277,425,54,1,DarkGreen);//µÚÊ®ÁÐÊúÏß
// for(i=0;i<8;i++) Dispsz16(djb[i][201],3,444,64+i*27,Green,Black); 

}



void Dispalm(void)
{//u8 i;
  ClearS(0,0,800,480,Magenta);
//----------------------------------±¨¾¯ÏÞ±í--------------------------------------------//
 LCD_DrawLine(0,50,708,0,Black);LCD_DrawLine(0,75,708,0,Black);LCD_DrawLine(0,100,708,0,Black);//µÚ1,2,3ÐÐºáÏß
 LCD_DrawLine(0,125,708,0,Black);LCD_DrawLine(0,150,708,0,Black);LCD_DrawLine(0,175,708,0,Black);//µÚ4,5,6ÐÐºáÏß
 LCD_DrawLine(0,200,708,0,Black);LCD_DrawLine(0,225,708,0,Black);LCD_DrawLine(0,250,708,0,Black);//µÚ7,8,9ÐÐºáÏß			 
 LCD_DrawLine(0,275,708,0,Black);LCD_DrawLine(0,300,708,0,Black);LCD_DrawLine(0,325,708,0,Black);//µÚ10,11,12ÐÐºáÏß	
 LCD_DrawLine(0,350,708,0,Black);LCD_DrawLine(0,375,708,0,Black);LCD_DrawLine(0,400,708,0,Black);//µÚ13,14,15ÐÐºáÏß	
 LCD_DrawLine(0,425,708,0,Black); LCD_DrawLine(0,450,708,0,Black);LCD_DrawLine(0,475,708,0,Black);//µÚ16,17,18ÐÐºáÏß	

 LCD_DrawLine(0,50,430,1,Black);//µÚÒ»ÁÐÊúÏß	
 Disphzs24(0xb5d8,51,1,Magenta,Red);Disphzs24(0xd6b7,51,25,Magenta,Red);//µØÖ·
 LCD_DrawLine(65,50,430,1,Black);//µÚ¶þÁÐÊúÏß
 Disphzs24(0xb5cd,51,66,Magenta,Blue);Disphzs24(0xcfde,51,90,Magenta,Blue);//µÍÏÞ
 Disphzs24(0xb8df,51,126,Magenta,Blue);Disphzs24(0xcfde,51,150,Magenta,Blue);//¸ßÏÞ
 LCD_DrawLine(177,50,430,1,Black);//µÚÈýÁÐÊúÏß
 Disphzs24(0xb5d8,51,178,Magenta,Red);Disphzs24(0xd6b7,51,202,Magenta,Red);//µØÖ·
 LCD_DrawLine(242,50,430,1,Black);//µÚËÄÁÐÊúÏß
 Disphzs24(0xb5cd,51,243,Magenta,Blue);Disphzs24(0xcfde,51,267,Magenta,Blue);//µÍÏÞ
 Disphzs24(0xb8df,51,303,Magenta,Blue);Disphzs24(0xcfde,51,327,Magenta,Blue);//¸ßÏÞ
 LCD_DrawLine(354,50,430,1,Black);//µÚÎåÁÐÊúÏß
 Disphzs24(0xb5d8,51,355,Magenta,Red);Disphzs24(0xd6b7,51,379,Magenta,Red);//µØÖ·
 LCD_DrawLine(419,50,430,1,Black);//µÚÁùÁÐÊúÏß
 Disphzs24(0xb5cd,51,420,Magenta,Blue);Disphzs24(0xcfde,51,444,Magenta,Blue);//µÍÏÞ
 Disphzs24(0xb8df,51,480,Magenta,Blue);Disphzs24(0xcfde,51,504,Magenta,Blue);//¸ßÏÞ
 LCD_DrawLine(531,50,430,1,Black);//µÚÆßÁÐÊúÏß
 Disphzs24(0xb5d8,51,532,Magenta,Red);Disphzs24(0xd6b7,51,556,Magenta,Red);//µØÖ·
 LCD_DrawLine(596,50,430,1,Black);//µÚ°ËÁÐÊúÏß
 Disphzs24(0xb5cd,51,597,Magenta,Blue);Disphzs24(0xcfde,51,621,Magenta,Blue);//µÍÏÞ
 Disphzs24(0xb8df,51,657,Magenta,Blue);Disphzs24(0xcfde,51,681,Magenta,Blue);//¸ßÏÞ
 LCD_DrawLine(708,50,430,1,Black);//µÚ¾ÅÁÐÊúÏß
}


void ProssRS485(void)
{
 if(trswait>0){Ur485_1=1; //Fa
               Utrs();
			         Ur485_1=0 ;
               trswait--;return;}//ÏìÓ¦ÊÂ¼þ
 //²é¿´ÍâÉè»Ø´ð×´Ì¬,
 if(inpzzhdl<inpzzhd){if(LedEQU==0){LedEQU=0x80;LEDbuffer[1]|=0x40;}else{LedEQU=0;LEDbuffer[1]&=0xbf;}
                      if(zxztb[zxnum]==0xff)HJalarm(21,0,134,0);
                      zxztb[zxnum]=0;
					           } 
 else{if(zxztb[zxnum]!=0xff){zxztb[zxnum]++;//tchk´ÎÎÞ»ØÖµ±¨¹ÊÕÏ
                             if(zxztb[zxnum]>60){zxztb[zxnum]=0xff;HJalarm(16,0,134,0);}}               
	   }
 inpzzhdl=inpzzhd;
 //Ñ²¼ì
 //RS485_down(0x7e,lxnum,0x4b,0,0,0,0,0);//²ãÏÔ
 RS485_down(0x7d,zxnum,0x6d,0,0,0,0,0);//×ÜÏßÅÌºÍ×¨ÏßÅÌ	
 zxnum++;if(zxnum>zxztb[0])	zxnum=0;								 
 Ur485_1=1;  //Fa
 Utrs();
 Ur485_1=0;
 return;
}
	

void Utrs(void)
{//u8 i=0;
 USART2Write(strbuf[wtsum],12);//for(i=0;i<12;i++)  USART1_SendByte(strbuf[wtsum][i]); 
 while (!(USART2->SR & USART_FLAG_TC));	 
 if(wtsum<31)wtsum++;
 if(wtsum==tsum){wtsum=0;tsum=0;}
 return;
}


//CRT
void RS485_down(u8 equ,u8 mac,u8 alm,u8 dll,u8 daa,u8 nam,u8 flo,u8 zon)
{ u8 i;
  for(i=0;i<12;i++)strbuf[tsum][i]=0;
  switch(alm){case 0:
              case 1:alm=0x42;break;//»ð¾¯
              case 2:
              case 3:alm=0x63;break;//Æô¶¯
              case 4:alm=0x65;break;//Í£Ö¹
              case 5:alm=0x61;break;//·´À¡
              case 6:alm=0x6b;break;//³·Ïû
              case 16:alm=0x67;break;//¹ÊÕÏ
              case 21:alm=0x69;break;//¹ÊÕÏ»Ö¸´
              case 32:alm=0x51;break;//ÆÁ±Î
              case 33:alm=0x53;break;//½â³ý
              default:break;}
   strbuf[tsum][0]=0xfd;strbuf[tsum][1]=equ;strbuf[tsum][2]=mac;
   strbuf[tsum][3]=alm;strbuf[tsum][4]=dll;strbuf[tsum][5]=daa;
   strbuf[tsum][6]=nam;strbuf[tsum][7]=flo;strbuf[tsum][8]=zon;strbuf[tsum][9]=0;strbuf[tsum][10]=0;
   strbuf[tsum][11]=0xfe;
   if(tsum<31){tsum++;if(alm!=0x4b)trswait++;}
}



void HLCX_down(u8 dllcx,u8 daacx,u8 dllalm,u8 daa,u8 nam,u8 flo,u8 zon)
{
 lpcxb[lpcxfg][0]=0xfe;lpcxb[lpcxfg][1]=daacx;lpcxb[lpcxfg][2]=dllalm;lpcxb[lpcxfg][3]=daa;
 lpcxb[lpcxfg][4]=nam;lpcxb[lpcxfg][5]=flo;lpcxb[lpcxfg][6]=zon;lpcxb[lpcxfg][7]=0;lpcxb[lpcxfg][8]=0;
 lpcxb[lpcxfg][9]=1;
 if(lpcxfg<59)lpcxfg++;
}



void ProssQYJ(void)
{
// if(inpzzhdl<inpzzhd){if(LedJZ==0){LedJZ=0x80;LedTX_on();}
// else{LedJZ=0;LedTX_off();}}//ÁªÍøµÆÉÁË¸
 inpzzhdl=inpzzhd; 
 RS485_up(0x7a,1,0x4b,0,0,0,0,0);
 Ur485_2=0 ;//·¢ËÍ
 Utrsup();
 Ur485_2=1 ;//½ÓÊÕ
 return;
}
	

void Utrsup(void)
{//u8 i=0;
 USART6Write(strbup[wtsumup],12);
 while (!(USART6->SR & USART_FLAG_TC));	 
 if(wtsumup<31)wtsumup++;
 if(wtsumup==tsumup){wtsumup=0;tsumup=0;}
 return;
}



//ÉÏ´«¼¯ÖÐ»ú
void RS485_up(u8 equ,u8 mac,u8 alm,u8 dll,u8 daa,u8 nam,u8 flo,u8 zon)
{ u8 i;
  for(i=0;i<12;i++)strbup[tsumup][i]=0;
  strbup[tsumup][0]=0xfd;strbup[tsumup][1]=equ;strbup[tsumup][2]=mac;
  strbup[tsumup][3]=alm;strbup[tsumup][4]=dll;strbup[tsumup][5]=daa;
  strbup[tsumup][6]=nam;strbup[tsumup][7]=flo;strbup[tsumup][8]=zon;strbup[tsumup][9]=0;strbup[tsumup][10]=0;
  strbup[tsumup][11]=0xfe;
  if(tsumup<31){tsumup++;if(alm!=0x4b)trswaitup++;}
}


void XCBC(void)
{u8 k; //u16 x,y;
 //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); 
 LedXT_on();//ÏµÍ³¹ÊÕÏµÆÁÁ
loop1:Disptu(25,Magenta,0,0); 
loop:k=0xff;key=0;Delay16s(1000,1000);
     do{KEYdeal();}while(key==0xff);
	   if(key<=0x21)k=Getnum(key);else goto loop;		
     if(k==0x13)
//loop3:
     { if(MenuBF>0){Menukey=MenuBF=MenuBX=0;goto loop1;}
            else{Menukey=MenuBF=MenuBX=0;
                 //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  
		        }
            LedXT_off();//ÏµÍ³¹ÊÕÏµÆÃð
		    Esc();
		    return;
		 }		
     goto loop;
}


u8 Getnum(u8 i)
{ u8 number=0xff;
  switch(i)
	{
    case 0x0:number=0;	                 
             break;
    case 0x1:if((MenuBF+Menukey)==0){MenuBF=1;Disptu(32,Magenta,0,0);}//ÏÔÊ¾±à³Ì
			       else{if(Menukey)number=1;}   
             break;
    case 0x2:if((MenuBF+Menukey)==0){MenuBF=2;Disptu(33,Magenta,0,0);}//Áª¶¯±à³Ì
			       else{if(Menukey)number=2;}  	
             break;
    case 0x3:if((MenuBF+Menukey)==0){MenuBF=3;Disptu(34,Magenta,0,0);}//Éè±¸Ãû³Æ
			       else{if(Menukey)number=3;} 	
             break;
    case 0x4:if((MenuBF+Menukey)==0){MenuBF=4;Disptu(35,Magenta,0,0);}//²ãÏÔ±à³Ì
			       else{if(Menukey)number=4;} 	
             break;
		case 0x5:if((MenuBF+Menukey)==0){MenuBF=5;Disptu(38,Magenta,0,0);}//×ÜÏß±à³Ì
			       else{if(Menukey)number=5;} 	
             break;				
    case 0x6:if(MenuBF>0){if(!Menukey){if(MenuBF==1){Menukey=1;XCXSBC();}
									                     if(MenuBF==2){Menukey=1;XCLDBC();}
			                                 if(MenuBF==3){Menukey=1;XCKRG();}
																			 if(MenuBF==4){Menukey=1;XCCXBC();}
				                               if(MenuBF==5){Menukey=1;XCZXBC();}
//									                     if(MenuBF==10)Analog();
																			  //if(MenuBF==11){Menukey=1;QDTest();}
                                      }
	                         else number=6;	   
				                }
             break;
    case 0x7:if(MenuBF>0){if(!Menukey){if(MenuBF==1){MenuBX=1;XCXSCX();}
									                     if(MenuBF==2){MenuBX=1;XCLDCX();} 
			                                 if(MenuBF==3){MenuBX=1;XCKRGCX();}
																			 if(MenuBF==4){MenuBX=1;XCCXCX();} 
																		   if(MenuBF==5){MenuBX=1;XCZXCX();} 
// 									                     if(MenuBF==10)AnalogCI(); 
																			 if(MenuBF==11){Menukey=1;RAMTest();}
                                      }
	                       else number=7;	   
			          	      }
             break;
    case 0x8:if(MenuBF>0){if(!Menukey){if(MenuBF==1){if((Menukey+MenuBX)==0)XCerase(1);} 
										                   if(MenuBF==2){if((Menukey+MenuBX)==0)XCerase(2);} 
                                       if(MenuBF==3){if((Menukey+MenuBX)==0)XCerase(3);} 
																			 if(MenuBF==4){if((Menukey+MenuBX)==0)XCerase(4);} 
																			 if(MenuBF==5){if((Menukey+MenuBX)==0)XCerase(5);} 																 
				                               if(MenuBF==11){MenuBX=1;TXTest();}				        
                                      }
	                        else number=8;	   
				                 }
             break;
    case 0x9:if(MenuBF>0){if(!Menukey){if(MenuBF==1){Menukey=1;Upload_PC(1);}
									                     if(MenuBF==2){Menukey=1;Upload_PC(2);}
			                                 if(MenuBF==3){Menukey=1;Upload_PC(3);}
																			 if(MenuBF==4){Menukey=1;Upload_PC(4);}
																			 //if(MenuBF==5){Menukey=1;Upload_PC(5);}
				                               if(MenuBF==11){Menukey=1;XTJL();}        
                                      }
	                        else number=9;	   
				                }						 
             break;
    case 0x13:number=i;Menukey=0;MenuBX=0;break;//ÍË³ö
    case 0x0d:number=i;break;                   //È¡Ïû
    case 0x0c:number=i;break;                   //ÏÂ
    case 0x0b:number=i;break;                   //ÉÏ
		case 0x11:number=0x3b;break;// ; 									
	  case 0x1b:number=0x26;break;// & 
	  case 0x1c:number=0x2b;break;// + 
	  case 0x1d:number=0x2a;break;// *       
	  case 0x1e:number=0x22;break;// ''
		case 0x1f:number=0x2c;break;// £¬
		case 0x20:if((MenuBF+Menukey)==0){MenuBF=11;Disptu(36,Magenta,0,0);}//Õû»ú²âÊÔ
		          else{if(Menukey)number=0x2d;}//-         
              break;												
	  case 0x21:if((MenuBF+Menukey)==0){MenuBF=10;Disptu(37,Magenta,0,0);}//ÏµÍ³¼ì²é
			        else{if(Menukey)number=0x3d;} // =                     
              break;					
    case 0x0f:number=i;break;   //¸´ÖÆ      
	  case 0x1a:number=i;break;   //±£´æ     
    default:number=0xff;break;
  }
	Delay16s(100,100);
  return(number);
}



void XCerase(u8 erase)
{//u16 i,j;u32 waddr;
 //iwdgtim=0;IWDG_ReloadCounter();
 ClearS(0,456,800,24,White);
 dstr=XCBCMenu;Dispstr(11,2,456,0,White,Black);//Çå¿Õ......
 if(erase==1) 
    {dstr=XCMenu;Dispstr(1,4,456,96,White,Red);
		 SPI_Flash_Erase_Block(10);
		 SPI_Flash_Erase_Block(11); 
    }
 if(erase==2) 
    {dstr=XCMenu;Dispstr(10,4,456,96,White,Blue);
		 SPI_Flash_Erase_Block(14);	
    }
 if(erase==3)
    {dstr=XCMenu;Dispstr(19,4,456,96,White,Magenta);
     SPI_Flash_Erase_Sector(159);
	  }
 if(erase==4)
   {dstr=XCMenu;Dispstr(28,4,456,96,White,Magenta);
		SPI_Flash_Erase_Sector(152);SPI_Flash_Erase_Sector(153);
	 }
 if(erase==5)
   {dstr=XCMenu;Dispstr(37,4,456,96,White,Magenta);
		SPI_Flash_Erase_Sector(156);
   }
 dstr=DJMenu;Dispstr(10,2,456,224,White,Black);
 return;
}



void XCXSBC(void)	//ÏÖ³¡ÏÔÊ¾±à³Ì
 {u8 i,j,m,dll,k=0xff,shuz[4],flo,zon,sec,qwm[20],fzqwm[20];
  u16 value[4],hz,cb=0,adr;
	u32 waddr; 
loop5:ClearS(0,0,800,480,Magenta);
	    Disphzs24(0xbbd8,30,48,Magenta,Blue);Disphzs24(0xc2b7,30,72,Magenta,Blue);Dispzf24('_',30,120,Magenta,Blue);
	    Disptu(29,Magenta,0,0);
	    for(i=0;i<4;i++)shuz[i]=0;j=0;cb=120;sec=0;
loopx:k=0xff;Delay16s(4000,3000);//Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key<0x20)k=Getnum(key);else goto loopx;
      if(k==0x13)return;
			if(k==0x0d)goto loop5;
	    if(k<=9){Dispsz24(k,1,30,cb,Magenta,Black);
				       shuz[j]=k;cb=cb+16;j++;//Òº¾§ÆÁÏÔÊ¾
				       if(j==2){dll=shuz[0]*10+shuz[1];goto loop6;} 
			        }
			goto loopx;
loop6:if(dll<=16)
	       {sec=160+(dll-1)*2;
          waddr=0xa0000+(dll-1)*6400;SPI_Flash_Read(downb,waddr,6400);
				 }
			else{if(dll>40&&dll<=48)//×¨ÏßÅÌ
				     {sec=208+dll-41;
              waddr=0xd0000+(dll-41)*4096;SPI_Flash_Read(downb,waddr,4096);
				     }
					 else goto loop5;
			    }	
			Disptu(40,Magenta,0,0);  //loopy:
loop3:ClearS(200,0,800,48,Magenta);m=0;cb=0;
      for(i=0;i<4;i++)value[i]=0;
	    for(i=0;i<20;i++)qwm[i]=0x20;
loop2:for(i=0;i<4;i++)shuz[i]=0;j=0;
      Dispzf24('_',200,cb,Magenta,White);
loop4:key=k=0xff;Delay16s(3000,3000);
      do{KEYdeal();Dispsz16(key,3,144,600,Magenta,Blue);}while(key==0xff);	
      if(key<0x20)k=Getnum(key);else goto loop4;
			if(k==0x13){for(hz=0;hz<16384;hz++)downb[hz]=0;return;} // loop:
      if(k==0x0d)goto loop3;
	    if(k==0x1a){//±£´æ
	                if(dll<=16){SPI_Flash_Erase_Sector(sec);SPI_Flash_Erase_Sector(sec+1);
				                      SPI_Flash_Write_NoCheck(downb,waddr,6400);
									           }
									else{SPI_Flash_Erase_Sector(sec);
				               SPI_Flash_Write_NoCheck(downb,waddr,4096);
									    }
				          Disptu(26,Magenta,200,680);Delay16s(5000,5000);
				          goto loop5;
			 	        }
	    if(k==0x0f){//¸´ÖÆ
		              for(i=0;i<20;i++)qwm[i]=fzqwm[i];
                  for(i=0;i<10;i++){hz=(qwm[2*i]<<8)+qwm[2*i+1];Disphzs24(hz,400,464+24*i,Magenta,Blue);}
                  goto loop4;}
      if(k==0x0c&&value[0]>0){//ÐÞ¸Ä
				                      adr=(value[0]-1)*32;
		                          if(value[1]<1000){downb[adr]=value[1];downb[adr+1]=flo;downb[adr+2]=zon;
		                                            for(i=0;i<20;i++){downb[adr+6+i]=qwm[i];fzqwm[i]=qwm[i];}
		 	                                         }
		                          else{for(i=0;i<32;i++)downb[adr+i]=0xff;}
                              goto loop3;}
      if(k<=9){Dispsz24(k,1,200,cb,Magenta,Black);shuz[j]=k;cb=cb+16;j++;
               if(j==4){cb=cb+16;
		                    if(m<4){if(m==0)value[m]=shuz[1]*100+shuz[2]*10+shuz[3];
													      else value[m]=shuz[0]*1000+shuz[1]*100+shuz[2]*10+shuz[3];
				                        if(m==3){
						                             ClearS(400,0,800,24,Magenta);Dispsz24(value[0],4,400,0,Magenta,Blue);
										                     Disprom(H_nam,0,value[1],400,64,Magenta,Blue);
													               if(value[2]>127){Dispzf24('-',400,232,Magenta,Blue);flo=value[2]-1000;Dispsz24(flo,1,400,248,Magenta,Blue);flo=flo+128;}
                                         else{flo=value[2];Dispsz24(flo,2,400,232,Magenta,Blue);}
																				 Dispzf24('F',400,268,Magenta,Blue);
                                         if(value[3]<128)zon=value[3];Dispsz24(zon,2,400,296,Magenta,Blue);Disphzs24(0xc7f8,400,328,Magenta,Blue);
												                }
						                   }						  
				                if(m>=4){
				                         qwm[2*(m-4)]=shuz[0]*10+shuz[1]+0xa0;qwm[2*m-7]=shuz[2]*10+shuz[3]+0xa0;//ÇøÎ»Âë£½(»úÄÚÂë-0xa0)Ê®½øÖÆ
                                 hz=(qwm[2*(m-4)]<<8)+qwm[2*m-7];Disphzs24(hz,400,352+24*(m-4),Magenta,Blue);cb=cb-80;ClearXY(200,348,24,96,Magenta);
						                    }
				                m++;
				                goto loop2;
                       }
               goto loop4;}
 }



void XCXSCX(void)
  {u8 i,dll=1,daa=1,nam,flo,zon=0;
   u16 adr,j;
loop4:daa=1;
		  if(dll<=16)SPI_Flash_Read(downb,(0xa0000+(dll-1)*6400),6400);//¶ÁÈëÏÔÊ¾±à³Ì
		  if(dll>=41&&dll<=48){SPI_Flash_Read(downb,(0xd0000+(dll-41)*4096),4096);
		                       for(j=4096;j<=8000;j++)downb[j]=0xff;
			                    }
loop1:ClearS(0,0,800,480,Magenta);
      i=0;
      for(;i<20;)
	       {adr=(daa-1)*32;
		      nam=downb[adr];flo=downb[adr+1];zon=downb[adr+2];
		      if(nam!=0xff)
		         {Dispsz24(dll,2,24*i,0,Magenta,Black);Dispsz24(daa,3,24*i,32,Magenta,Black); 
	            Disprom(H_nam,0,nam,24*i,80,Magenta,Black);
		          if((zon<128))  //(zon>=0)&&
								{ Dispsz24(zon,2,24*i,240,Magenta,Black);
							    Disphzs24(0xc7f8,24*i,272,Magenta,Black);
								}
			        if(flo<0xff){if(flo>128){flo-=128;Dispzf24('-',24*i,328,Magenta,Black);Dispsz24(flo,1,24*i,344,Magenta,Black);}
						               else Dispsz24(flo,2,24*i,328,Magenta,Black);
						               Dispzf24('F',24*i,360,Magenta,Black);
						              }
				      if(dll<=16)Disprom(H_rom,dll,daa,24*i,392,Magenta,Black);
							if(dll>=41&&dll<=48)Disprom(H_romzx,dll,daa,24*i,392,Magenta,Black);					
              i++;
		         }
		      daa++;if(daa==201)break;
	 	     }
			if(i==0)goto loop5; 
 loop:key=0;Delay16s(4000,3000);
      do{KEYdeal();}while((key==0xff));
				if(key==0x13)return;	//loop3:
      if(key==0x0c)
loop5:	  {if(daa<=200)goto loop1;
	  		   else{dll++;if(dll>16)dll=41;if(dll>48)dll=1;
	  					  goto loop4;}
	        }
      if(key==0x0b){dll=1;goto loop4;}
      goto loop;
}




void XCKRG(void)
{u8 i,j,k=0xff,m,shuz[4],qwm[16];
 u16 val,adr,hz,cb;
 SPI_Flash_Read(downb,0x9f000,4096);
 Disptu(42,Magenta,0,0);
 loop:ClearS(200,0,800,48,Magenta);
      m=0;cb=0;val=0;for(i=0;i<16;i++) qwm[i]=0x20;
loop2:for(i=0;i<4;i++)shuz[i]=0;j=0;
      Dispzf24('_',200,cb,Magenta,White);
loop4:k=0xff;Delay16s(4000,3000);//Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key<=0x1a)k=Getnum(key);else goto loop4;
			if(k==0x13){for(adr=0;adr<4096;adr++)downb[adr]=0;return;}  //loop3:
      if(k==0x0d)goto loop;
	    if(k==0x1a){//±£´æ
				          Disptu(27,Blue,200,480);
				          SPI_Flash_Erase_Sector(159);
				          SPI_Flash_Write_NoCheck(downb,0x9f000,4096);
				          Disptu(28,Blue,200,560);Disptu(26,Magenta,200,680);Delay16s(5000,5000);
				          goto loop;
	               }
      if(k==0x0c&&val<128){//ÐÞ¸Ä
		                       adr=val*16;
		                       for(i=0;i<16;i++)downb[adr+i]=qwm[i];
                           ClearS(400,0,800,24,Magenta);
				                   Dispsz24(val,4,400,0,Magenta,White);
				                   for(i=0;i<8;i++){hz=(qwm[2*i]<<8)+qwm[2*i+1];Disphzs24(hz,400,96+24*i,Magenta,Black);}
                           goto loop;}
      if(k<10){Dispsz24(k,1,200,cb,Magenta,White);shuz[j]=k;cb=cb+16;j++;
               if(j==4){if(m==0){val=shuz[1]*100+shuz[2]*10+shuz[3];cb=96;}
                        else{qwm[2*(m-1)]=shuz[0]*10+shuz[1]+0xa0;qwm[2*m-1]=shuz[2]*10+shuz[3]+0xa0;//ÇøÎ»Âë£½(»úÄÚÂë-0xa0)Ê®½øÖÆ
					   	               hz=(qwm[2*(m-1)]<<8)+qwm[2*m-1];Disphzs24(hz,224,cb-32,Magenta,Black);cb=cb+8;
												    }
                        m++;if(m<9)goto loop2;}
               goto loop4;}
	  goto loop4;
}



void XCKRGCX(void)
{u8 i,m=0;u16 cb,rb;
loop:ClearS(0,0,800,480,Magenta);//ÇåÆÁ
     rb=0;cb=0;
	   for(i=0;i<60;i++)
	       {Dispsz24(m,3,rb,cb,Magenta,White);Disprom(H_nam,0,m,rb,cb+48,Magenta,Blue);
			    rb=rb+24;if(rb==480){rb=0;cb=cb+260;}		 
		      m++;
		      if(m==128) break;} 
loop1:key=0;Delay16s(4000,3000);
      do{KEYdeal();}while((key==0xff));
			if(key==0x13)return;		//loop2:
      if(key==0x0c){if(m>=128) m=0;
				            goto loop;
				           }
      if(key==0x0b){m=0;goto loop;}
      goto loop1;
}



/*
×¢:(1)Ìõ¼þ0(&),Ìõ¼þ1(+),Ìõ¼þ2(*)×î¶à¿ÉÊäÈë16¸öÌõ¼þµØÖ·£¬×î¶à¿ÉÊä³ö16¸öÁª¶¯µØÖ·
(2)Ìõ¼þ3(¡®¡¯)¶àµã¶ÎÓë£¬Á½¶Î×ÜºÍ×î¶à¿ÉÊäÈë16¸öÌõ¼þµØÖ·(Ã¿¶ÎÖÁÉÙº¬1¸öµØÖ·£¬Á½¶Î³¤¶ÈËæÒâ×éºÏ)£¬×î¶à¿ÉÊä³ö16¸öÁª¶¯µØÖ·£¬µÚÒ»¶ÎÓëµÚ¶þ¶ÎÖ®¼äÓÃ0xff·Ö¸ô
*/
void XCLDBC(void)			//ÏÖ³¡Áª¶¯±à³Ì
{u8 i,j,m,n,k=0xff,lg,shuz[5],ld[70];
 u16 pf,pn,rb,cb,num,t;
 u32 waddr;
loop9:for(i=0;i<70;i++)ld[i]=0;
	    for(t=0;t<16384;t++)downb[t]=0;
      SPI_Flash_Read(downb,0xef000,1024);
	    num=downb[1022]<<8;num=num+downb[1023];
      if(num>1000){num=0;pf=0;downb[0]=0;downb[1]=0;downb[1022]=0;downb[1023]=0;}//ÎÞ±à³Ì
      else {pf=downb[num*2]<<8;pf=pf+downb[num*2+1];}//Ö¸Õë
	    num++;t=0;
loop3:Disptu(43,Magenta,0,0);Dispsz24(num,3,0,0,Magenta,Blue);Dispzf24('_',48,56,Magenta,White);
      m=0;
//¹ØÏµ
loop4:k=0xff;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      k=Getnum(key);
	    if(k==0x1a&&t>0)goto loop8;//±£´æ
	    if(k==0x13){for(t=0;t<16384;t++)downb[t]=0;return;}
	    if(k==0x0d)goto loop3;
	    if(k==0x26||k==0x2b||k==0x2a||k==0x22){Dispzf24(k,48,56,Magenta,Black);ld[0]=k;goto loop1;}
	    goto loop4;
//Ìõ¼þ
loop1:rb=120;cb=16;lg=0;m=1;
loop2:for(i=0;i<5;i++)shuz[i]=0;j=0;Dispzf24('_',rb,cb,Magenta,White);
loop:key=k=0xff;Delay16s(4000,3000);//Delay16s(3000,3000);
     do{KEYdeal();}while(key==0xff);
     k=Getnum(key);
	   if(k==0x1a&&t>0)goto loop8;//±£´æ
	   if(k==0x13){for(t=0;t<16384;t++)downb[t]=0;return;}
	   if(k==0x0d)goto loop3;
	   if(k<10)Dispsz24(k,1,rb,cb,Magenta,White);
     else{if(k==0x2c||k==0x2d){Dispzf24(k,rb,cb,Magenta,White);cb+=16;if(cb>=784){rb=rb+24;cb=16;}
	                             if(k==0x2d)ld[m-2]=ld[m-2]+0x80; 
														  }
          if(k==0x22){ld[m]=0xff;m++;rb=rb+24;cb=16;Dispzf24(k,rb,0,Magenta,White);}//¶àµã¶ÎÓë
		      if(k==0x3d){ld[m]=0xfb;m++;goto loop5;}//=
          goto loop2;}
     shuz[j]=k;j++;cb+=16;if(cb>=784){rb=rb+24;cb=16;}
     if(j==5){ld[m]=shuz[0]*10+shuz[1];ld[m+1]=shuz[2]*100+shuz[3]*10+shuz[4];//Ìõ¼þµØÖ·
	            m=m+2;lg++;if(lg>15){ld[m]=0xfb;m++;goto loop5;}
              goto loop2;}
     goto loop;
//Áª¶¯
loop5:ClearXY(192,56,48,700,Magenta);rb=192;cb=16;lg=0;n=m;   
loop6:for(i=0;i<5;i++)shuz[i]=0;j=0;Dispzf24('_',rb,cb,Magenta,Black);
loop7:key=k=0xff;Delay16s(4000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key<=0x21)k=Getnum(key);else goto loop7;
      if(k==0x13){for(t=0;t<16384;t++)downb[t]=0;return;}
      if(k==0x0d)goto loop5; 
	    if(k==0x0b)goto loop3; 
	    if(k==0x1a&&t>0){//±£´æ
	             loop8:waddr=0xe0000+pf;SPI_Flash_Write_NoCheck((downb+2048),waddr,t);
					           SPI_Flash_Erase_Sector(239);SPI_Flash_Write_NoCheck(downb,0xef000,4096);
				             Disptu(26,Magenta,200,680);Delay16s(5000,5000);
				             goto loop9;
			                }
      if(k==0x0c)//»º´æ
							{ld[n]=0xfc;n++; //loop13:
				         pn=pf+t+n;
				         downb[num*2]=pn>>8;downb[num*2+1]=pn&0x00ff;
				         downb[1022]=num>>8;downb[1023]=num&0x00ff;
				         for(i=0;i<n;i++)downb[2048+t+i]=ld[i];t=t+n;//½«´ËÌõ±à³ÌÐ´Èë»º´æÖÐ
				         num++;
                 goto loop3;}
	  if(lg>15)goto loop6;        //×Ô¶¯Ìø×ªµÈ´ý±£´æ 
      else{if(k<10)Dispsz24(k,1,rb,cb,Magenta,Black);
	         else{if(k==0x2c||k==0x2d){Dispzf24(k,rb,cb,Magenta,Black);cb+=16;if(cb>=784){rb=rb+24;cb=16;}
	                                   if(k==0x2d)ld[n-2]=ld[n-2]+0x80;}
			          goto loop6;
			         }
	         shuz[j]=k;cb=cb+16;j++;if(cb>=784){rb=rb+24;cb=16;}//Òº¾§ÆÁÏÔÊ¾
           if(j==5){ld[n]=shuz[0]*10+shuz[1];ld[n+1]=shuz[2]*100+shuz[3]*10+shuz[4];
				             n=n+2;lg++;
				            goto loop6;
                   }
           goto loop7;}
 }




void XCLDCX(void)  
  {u8 i,k,x,y,n,buf[70],lg;
   u16 num,sum,rb,cb,pf,pn,m;//,val,v;
   u32 waddr;
   for(m=0;m<1024;m++)downb[m]=0;m=0;
   SPI_Flash_Read(downb,0xef000,1024);
   sum=downb[1022]<<8;sum=sum+downb[1023];if(sum>200) sum=0;//ÎÞ±à³Ì
   if(sum==0)return;  
loop2:num=0;m=0;
loop1:ClearS(0,0,800,480,Magenta);dstr=XCMenu;Dispstr(10,4,0,0,Magenta,Blue);Dispsz24(sum,3,0,96,Magenta,White);
	    rb=24;cb=32;
	  do{num++;Dispsz24(num,3,rb,0,Magenta,Blue);lg=0;
	     pf=downb[m]<<8;pf=pf+downb[m+1];  
	     pn=downb[m+2]<<8;pn=pn+downb[m+3];
		   k=pn-pf;
		   waddr=0xe0000+pf;SPI_Flash_Read(buf,waddr,k);	   
		   rb=rb+24;Dispzf24(buf[0],rb,0,Magenta,White);//ÏÔÊ¾Ìõ¼þ
		   for(i=1;i<70;)
			   {x=buf[i];i++;if(x==0xfb)break;//Ìõ¼þ½áÊø
			    if((x==0xff)&&(buf[0]==0x22)){rb=rb+24;cb=32;Dispzf24(0x22,rb,0,Magenta,Black);continue;}
			    y=buf[i];i++;
          if(x>0x80){x=x-0x80;n=0x2d;} else n=0x2c; 
					//val=x*1000+y;
					Dispsz24(x,2,rb,cb,Magenta,White);Dispsz24(y,3,rb,cb+32,Magenta,White);cb+=80;
					Dispzf24(n,rb,cb,Magenta,White);cb+=16;	
          lg++;					
			    if(cb>=800){if(lg<=15){rb=rb+24;cb=32;}if(rb>=480){num--;goto loop;}}//Ã¿ÐÐÏÔÊ¾8¸öÁª¶¯Ìõ¼þ
			   }
		   rb=rb+24;cb=32;Dispzf24(0x3d,rb,0,Magenta,Black);lg=0; //=
		   for(;i<k;)
			  {x=buf[i];i++;if(x==0xfc)break;//½áÊø
			   y=buf[i];i++;
         if(x>0x80){x=x-0x80;n=0x2d;} else n=0x2c; //-	,
				 Dispsz24(x,2,rb,cb,Magenta,Black);Dispsz24(y,3,rb,cb+32,Magenta,Black);cb+=80;
         Dispzf24(n,rb,cb,Magenta,Black);cb+=16;
				 lg++;	
			   if(cb>=800){if(lg<=15){rb=rb+24;cb=32;}if(rb>=480){num--;goto loop;}}//Ã¿ÐÐÏÔÊ¾8¸öÁª¶¯µØÖ·
			  }
		 rb=rb+24;cb=32;if(rb>=480){num--;goto loop;}
	   m=m+2;}while(num<sum);		 
loop:key=0;Delay16s(4000,3000);
      do{KEYdeal();}while(key==0xff);
			if(key==0x13)return; //loop3:
      if(key==0x0c){if(num<sum)goto loop1;
	                goto loop2;
	               }
      if(key==0x0b)goto loop2;
      goto loop;
}
	


void XCCXBC(void)		//ÏÖ³¡  ²ãÏÔ±à³Ì
{u8 i,j,m,k=0xff,shuz[5],value[6];
 u16 adr,cb;
 for(adr=0;adr<16384;adr++)downb[adr]=0;
 SPI_Flash_Read(downb,0x98000,16384);
 Disptu(53,Magenta,0,0);
loop3:ClearS(96,0,800,48,Magenta);
	    m=0;cb=120;for(i=0;i<6;i++)value[i]=0;  
loop2:for(i=0;i<5;i++)shuz[i]=0;j=0;  
loop4:Dispzf24('_',96,cb,Magenta,White);         
loop1:k=0xff;Delay16s(4000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key<=0x1a)k=Getnum(key);else goto loop1;
			if(k==0x13){for(adr=0;adr<16384;adr++)downb[adr]=0; //loop:
	               return;}
     if(k==0x0d)goto loop3;
	   if(k==0x1a){//±£´æ
                 SPI_Flash_Erase_Sector(152);SPI_Flash_Erase_Sector(153);
				         SPI_Flash_Write_NoCheck(downb,0x98000,6400);
				         Disptu(26,Magenta,200,680);Delay16s(5000,5000);
				         goto loop3;
			          }
     if(k==0x0c&&value[0]>0)
			         {//»º´æ
                adr=(value[0]-1)*1024+(value[1]-1)*4;
		            downb[adr]=value[2];downb[adr+1]=value[3];downb[adr+2]=value[4];downb[adr+3]=value[5];				      
				        Dispsz24(value[0],2,240,120,Magenta,White);Dispsz24(value[1],3,240,152,Magenta,White);Dispzf(0x1a,240,200,Magenta,Black);
				        Dispsz24(value[2],2,240,216,Magenta,Blue);Dispsz24(value[3],3,240,248,Magenta,Blue);
								Dispsz24(value[4],2,240,350,Magenta,Yellow);Dispsz24(value[5],3,240,382,Magenta,Yellow); 
                goto loop3;}
     else{if(m==6)goto loop1;
			    Dispsz24(k,1,96,cb,Magenta,Black);shuz[j]=k;cb=cb+16;j++;//Òº¾§ÆÁÏÔÊ¾
			    if(j==2){cb=cb+40;goto loop4;}     
          if(j==5){value[m]=shuz[0]*10+shuz[1];value[m+1]=shuz[2]*100+shuz[3]*10+shuz[4];	    
				           m=m+2;cb=cb+24;
				           goto loop2;
                  }
          goto loop1;}
 }


void XCCXCX(void)
  {u8 i,dll,daa,cx1,cx2,cx11,cx22;
   u16 adr=0,rb=0,cb;
   SPI_Flash_Read(downb,0x98000,16384);
loop2:dll=1;daa=1;
loop1:ClearS(0,0,800,480,Magenta);
	    rb=0;cb=0;
	    for(i=0;i<60;i++)
	       {Dispsz24(dll,2,rb,cb,Magenta,Black);Dispsz24(daa,3,rb,cb+32,Magenta,Black);
					Dispzf(0x1a,rb,cb+80,Magenta,White);
					adr=(dll-1)*1024+(daa-1)*4; 
		      cx1=downb[adr];cx11=downb[adr+1];cx2=downb[adr+2];cx22=downb[adr+3];
			    if(cx11>0&&cx11<=200){Dispsz16(cx1,2,rb,cb+96,Magenta,Blue);Dispsz16(cx11,3,rb,cb+112,Magenta,Blue);}      //µÚÒ»¸ö²ãÏÔ
		      if(cx22>0&&cx22<=200){Dispsz16(cx2,2,rb+16,cb+96,Magenta,Yellow);Dispsz16(cx22,3,rb+16,cb+112,Magenta,Yellow);}//µÚ¶þ¸ö²ãÏÔ
		    	cb=cb+152;if(cb>=750){cb=0;rb=rb+40;}		 
		      daa++;
		      if(daa>=201) break;
         } 
loop:key=0;Delay16s(4000,3000);
     do{KEYdeal();}while(key==0xff);
			if(key==0x13)return; //loop3:
      if(key==0x0c){if(daa>=201){dll++;daa=1;if(dll>16)goto loop2;}
	                  goto loop1;
	                 }
      if(key==0x0b)goto loop2;
      goto loop;
}


void Download_CX(void) //ÏÖ³¡  ²ãÏÔÏÂÔØ
{u16 cb=352;
 u32 i=0;
 //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
 TIM_Cmd(TIM1, DISABLE);//½ûÖ¹¶¨Ê±Æ÷1
//ÏÂÔØÉè±¸Ãû³ÆÍ·ÎÄ¼þ
   ClearS(180,256,300,16,Magenta);
   dstr=XCMenu;Dispstr(19,4,180,256,Magenta,Black);
   dstr=XCCXMenu;Dispstr(16,2,180,304,Magenta,Black);
   downb[0]=0xfd;downb[1]=0x7e;downb[2]=0x3f;downb[3]=0x4c;
   downb[4]=20;downb[5]=48;
   SPI_Flash_Read(downb+6,0x9f000,2048);
   for(i=0;i<2048;i++){if(downb[i+6]==0xff)downb[i+6]=0x20;}
   downb[2054]=0xfe;
   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
   i=0;
//   Ur485_1=1;
loop:USART1_SendByte(downb[i]);
     i++;
	 if((i%1000)==0){//iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
	                 Dispzf(22,180,cb,Magenta,Black);cb+=8;}
     if(i<2056)goto loop;
//   Ur485_1=0;
//   iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
//   Delay16s(7000,7000);
//   iwdgtim=0;IWDG_ReloadCounter();
//   Delay16s(7000,7000);
//   iwdgtim=0;IWDG_ReloadCounter();
//   Delay16s(7000,7000);
//   iwdgtim=0;IWDG_ReloadCounter();
//   Delay16s(7000,7000);
//   iwdgtim=0;IWDG_ReloadCounter();
//ÏÂÔØÏÔÊ¾±à³ÌÎÄ¼þ
   ClearS(180,256,300,16,Magenta);
   dstr=XCMenu;Dispstr(1,4,180,256,Magenta,Black);
   dstr=XCCXMenu;Dispstr(16,2,180,304,Magenta,Black);
   downb[0]=0xfd;downb[1]=0x7e;downb[2]=0x40;downb[3]=0x4c;
   downb[4]=65;downb[5]=36;    //downb[4]=655;?
   SPI_Flash_Read(downb+6,0xa0000,65535); //?
   for(i=0;i<65536;i++){if(downb[i+6]==0xff)downb[i+6]=0x20;}
   downb[65542]=0xfe;
   i=0;
//   Ur485_1=0;
loop1:USART1_SendByte(downb[i]);
      i++;
	  if((i%1000)==0){//iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
	                  Dispzf(22,180,cb,Magenta,Black);cb+=8;}
      if(i<65542)goto loop1;
//   Ur485_1=1;
 for(i=0;i<8010;i++)downb[i]=0;
 TIM_Cmd(TIM1, ENABLE);
 dstr=DJMenu;Dispstr(9,2,350,256,Magenta,Black);
 return;
}


void XCZXBC(void)		//ÏÖ³¡  ×ÜÏßÅÌ±à³Ì
{u8 i,j,m,k=0xff,shuz[5],value[6];
 u16 adr,cb;
 for(adr=0;adr<8192;adr++)downb[adr]=0;
 SPI_Flash_Read(downb,0x9c000,8192);
 Disptu(54,Magenta,0,0);
loop3:ClearS(96,0,800,48,Magenta);
	    m=0;cb=120;for(i=0;i<6;i++)value[i]=0;  
loop2:for(i=0;i<5;i++)shuz[i]=0;j=0;   
loop4:Dispzf24('_',96,cb,Magenta,White);         
loop1:k=0xff;Delay16s(4000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key<=0x1a)k=Getnum(key);else goto loop1;
//loop
			if(k==0x13){for(adr=0;adr<8192;adr++)downb[adr]=0;
	               return;}
     if(k==0x0d)goto loop3;
	   if(k==0x1a){//±£´æ
                 SPI_Flash_Erase_Sector(156);
				         SPI_Flash_Write_NoCheck(downb,0x9c000,4096);
				         Disptu(26,Magenta,200,680);Delay16s(5000,5000);
				         goto loop3;
			          }
     if(k==0x0c&&value[0]>0)
			         {//»º´æ
                adr=(value[0]-1)*512+(value[1]-1)*2;
		            downb[adr]=value[2];downb[adr+1]=value[3];			      			      
				        Dispsz24(value[0],2,240,120,Magenta,White);Dispsz24(value[1],3,240,152,Magenta,White);Dispzf(0x1a,240,200,Magenta,Black);
				        Dispsz24(value[2],2,240,216,Magenta,Blue);Dispsz24(value[3],3,240,248,Magenta,Blue);
                goto loop3;}
     else{if(m==4)goto loop1;
			    Dispsz24(k,1,96,cb,Magenta,Black);shuz[j]=k;cb=cb+16;j++;//Òº¾§ÆÁÏÔÊ¾
			    if(j==2){cb=cb+40;goto loop4;}     
          if(j==5){value[m]=shuz[0]*10+shuz[1];value[m+1]=shuz[2]*100+shuz[3]*10+shuz[4];	    
				           m=m+2;cb=cb+24;
				           goto loop2;
                  }
          goto loop1;}
 }


void XCZXCX(void)
{  u8 i,dll,daa,zx1,zx2;
   u16 adr=0,rb=0,cb;
   SPI_Flash_Read(downb,0x9c000,8192);
loop2:dll=1;daa=1;
loop1:ClearS(0,0,800,480,Magenta);
	    rb=0;cb=0;
	    for(i=0;i<100;i++)
	       {Dispsz24(dll,2,rb,cb,Magenta,Black);Dispsz24(daa,3,rb,cb+32,Magenta,Black);
					Dispzf(0x1a,rb,cb+80,Magenta,White);
					adr=(dll-1)*512+(daa-1)*2; 
		      zx1=downb[adr];zx2=downb[adr+1];
			    if(zx2>0&&zx2<=200){Dispsz16(zx1,2,rb,cb+96,Magenta,Blue);Dispsz16(zx2,3,rb,cb+112,Magenta,Blue);}//×ÜÏßÅÌ°´¼ü
		    	cb=cb+152;if(cb>=750){cb=0;rb=rb+24;}		 
		      daa++;
		      if(daa>=201) break;
         } 
loop:key=0;Delay16s(4000,3000);
     do{KEYdeal();}while(key==0xff);
//loop3:
		 if(key==0x13)return;
      if(key==0x0c){if(daa>=201){dll++;daa=1;if(dll>16)goto loop2;}
	                  goto loop1;
	                 }
      if(key==0x0b)goto loop2;
      goto loop;
}




	
void RAMTest(void)
{u8 buf[12],shuz[8]; //daa,dll,
 u16 i,j,t,rb,cb,wifi,cmd;
 u32 addr;
 //u16 x,y;////////////////////////////////
 ClearS(0,0,800,480,Magenta);Disptu(47,Magenta,0,0);
 for(i=0;i<2400;i++)downb[i]=0;
 downb[0]=llpp;downb[1]=2;downb[2]=0;downb[3]=Boma;downb[4]=printer;downb[5]=bdjc;downb[6]=sgjc;downb[7]=CRTxj;downb[8]=0;//downb[9]=zxlock;
 downb[10]=autofg;downb[11]=machine;downb[12]=machine;downb[13]=hjzz;downb[14]=gzzz;downb[15]=xspbs;downb[16]=ctime[3];downb[17]=ctime[2];
 downb[18]=hjjlzz>>8;downb[19]=hjjlzz&0x00ff;downb[20]=hjjltrue>>8;downb[21]=hjjltrue&0x00ff;downb[22]=0;downb[23]=0;downb[24]=0;downb[25]=0;
 downb[26]=tjlzz>>8;downb[27]=tjlzz&0x00ff;downb[28]=tjltrue>>8;downb[29]=tjltrue&0x00ff;
 downb[30]=pass[0]>>8;downb[31]=pass[0]&0x00ff;downb[32]=pass[1]>>8;downb[33]=pass[1]&0x00ff;downb[34]=pass[2]>>8;downb[35]=pass[2]&0x00ff;
 for(i=0;i<18;i++)downb[36+i]=0;//±£Áô0036~0053
 for(i=0;i<200;i++)downb[54+i]=cxb[i];//Áª¶¯±í0054~0143
 for(i=0;i<8;i++){for(j=0;j<=200;j++)downb[200+i*200+j]=djb[i][j];downb[200+i*200+201]=djb[i][201];}  //»ØÂ·µÇ¼Ç±í0144~0719
loop:j=0;
loop1:ClearS(0,0,800,480,Magenta);Disptu(47,Magenta,0,0);    
      rb=24;cb=0;
      for(i=0;i<342;i++){if((j%18)==0){Dispsz24(j,4,rb,0,Magenta,Black);cb=cb+80;}//Ã¿´Î»»ÐÐÏÔÊ¾´ËÐÐÊ×µØÖ·,Ã¿ÐÐÏÔÊ¾18¸ö×´Ì¬Öµ
                         DispszHEX(downb[j],rb,cb,Magenta,White);
					               cb=cb+40;if(cb>=800){rb=rb+24;cb=0;}
						             j++;if(j==342)	break;
                        }	  
loop3:key=0;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key==0x13){for(i=0;i<1200;i++)downb[i]=0;return;}
      if(key==0x0c){if(j<1025)goto loop1;
			              goto loop;
	                 }
      if(key==0x0b)goto loop;
			if(key==0x0f)goto loop2;//ÃüÁî						 
      goto loop3;
loop2://ÄÚ²¿µ÷ÊÔ	
			Disptu(47,Magenta,0,0);Dispzf24('_',0,264,Magenta,White); 							 
		  for(i=0;i<8;i++)shuz[i]=0;j=0;cb=264;					 
loop4:key=0;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key==0x13)goto loop;
      if(key==0x0c)goto loop5;	
	    if(key==0x0d)goto loop2;
		  if(key<10){Dispsz24(key,1,0,cb,Magenta,White);shuz[j]=key;cb=cb+16;j++;
			           if(j==4)cb=448;Dispzf24('_',0,cb,Magenta,White);	
                 if(j==8){cmd=shuz[0]*1000+shuz[1]*100+shuz[2]*10+shuz[3];wifi=shuz[4]*1000+shuz[5]*100+shuz[6]*10+shuz[7];}
                }						
      goto loop4;							 								 
loop5:for(i=0;i<16384;i++)downb[i]=0;
if(cmd==1&&wifi==3333)  //  ÉÏ´«»ð¾¯¼ÇÂ¼²éÑ¯
   {Disphzs24(0xc9cf,0,550,Magenta,Black);Disphzs24(0xb4ab,0,574,Magenta,Black);Disphzs24(0xbbf0,0,598,Magenta,Red);//ÉÏ´«»ð¾¯¼ÇÂ¼
    Disphzs24(0xbeaf,0,622,Magenta,Red);Disphzs24(0xbcc7,0,646,Magenta,Red);Disphzs24(0xc2bc,0,670,Magenta,Red);
    if(hjjltrue>hjjlzz)addr=0x80000+(hjjltrue-hjjlzz)*8;else addr=0x80000;
    t=hjjlzz*8;
    downb[0]=0xfd;downb[1]=0x80;downb[2]=0;downb[3]=0x4c;downb[4]=t/100;downb[5]=t%100;
    SPI_Flash_Read(downb+6,addr,t);
    downb[t+6]=0xfe;
		goto loop6;
   }
if(cmd==2&&wifi==3333) //Áª¶¯¼ÇÂ¼²éÑ¯
  {Disphzs24(0xc9cf,0,550,Magenta,Black);Disphzs24(0xb4ab,0,574,Magenta,Black);Disphzs24(0xc1aa,0,598,Magenta,Blue);//ÉÏ´«Áª¶¯¼ÇÂ¼
   Disphzs24(0xb6af,0,622,Magenta,Blue);Disphzs24(0xbcc7,0,646,Magenta,Blue);Disphzs24(0xc2bc,0,670,Magenta,Blue);
   if(ldjltrue>ldjlzz)addr=0x84000+(ldjltrue-ldjlzz)*8;else addr=0x84000;
   t=ldjlzz*8;
   downb[0]=0xfd;downb[1]=0x80;downb[2]=0;downb[3]=0x4c;downb[4]=t/100;downb[5]=t%100;
   SPI_Flash_Read(downb+6,addr,t);
   downb[t+6]=0xfe;
   goto loop6;
  }
if(cmd==3&&wifi==3333) //¹ÊÕÏ¼ÇÂ¼²éÑ¯
  {Disphzs24(0xc9cf,0,550,Magenta,Black);Disphzs24(0xb4ab,0,574,Magenta,Black);Disphzs24(0xc6e4,0,598,Magenta,Yellow);//ÉÏ´«ÆäËü¼ÇÂ¼
   Disphzs24(0xcbfc,0,622,Magenta,Yellow);Disphzs24(0xbcc7,0,646,Magenta,Yellow);Disphzs24(0xc2bc,0,670,Magenta,Yellow);
   if(tjltrue>tjlzz)addr=0x88000+(tjltrue-tjlzz)*8;else addr=0x88000;
   t=tjlzz*8;
   downb[0]=0xfd;downb[1]=0x80;downb[2]=0;downb[3]=0x4c;downb[4]=t/100;downb[5]=t%100;
   SPI_Flash_Read(downb+6,addr,t);
   downb[t+6]=0xfe;
loop6:i=0;
	 while(i<(t+7)){USART1_SendByte(downb[i]);i++;Dispsz24(i,4,0,700,Magenta,Black);
				          if((i%1000)==0){//iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
                                  Delay16s(10000,10000);
                                  //iwdgtim=0;IWDG_ReloadCounter();
                                  Delay16s(10000,10000);
							                    //iwdgtim=0;IWDG_ReloadCounter();
					                       }
						      while (!(USART1->SR & USART_FLAG_TC));
				         }    
   }
//Çå³ýÃüÁî				
if(cmd==37&&wifi==5381)
	  {dstr=XCBCMenu;Dispstr(11,2,0,550,Magenta,Black);Disphzs24(0xb4ab,0,598,Magenta,Black);	
	   Disphzs24(0xbbf0,0,598,Magenta,Red);Disphzs24(0xbeaf,0,622,Magenta,Red);//Çå³ý»ð¾¯¼ÇÂ¼	
		 dstr=HZMenu;Dispstr(11,2,0,646,Magenta,Black);
		 SPI_Flash_Erase_Sector(128);SPI_Flash_Erase_Sector(129);
	   SPI_Flash_Read(buf,0x8c000,12);	
		 buf[0]=0xff;buf[1]=0xff;buf[2]=0xff;buf[3]=0xff;	
     SPI_Flash_Erase_Sector(140);
		 SPI_Flash_Write_Page(buf,0x8c000,12);	
     hjjlzz=0;hjjltrue=0;
	  } 
if(cmd==38&&wifi==5381)
   {dstr=XCBCMenu;Dispstr(11,2,0,550,Magenta,Black);
		Disphzs24(0xc1aa,0,598,Magenta,Blue);Disphzs24(0xb6af,0,622,Magenta,Blue);//Çå³ýÁª¶¯¼ÇÂ¼ 
	  dstr=HZMenu;Dispstr(11,2,0,646,Magenta,Black);
	  SPI_Flash_Erase_Sector(132);SPI_Flash_Erase_Sector(133);
    SPI_Flash_Read(buf,0x8c000,12);	
		buf[4]=0xff;buf[5]=0xff;buf[6]=0xff;buf[7]=0xff;
    SPI_Flash_Erase_Sector(140);
	  SPI_Flash_Write_Page(buf,0x8c000,12);	
    ldjlzz=0;ldjltrue=0;
	 }		
if(cmd==39&&wifi==5381) 
   {dstr=XCBCMenu;Dispstr(11,2,0,550,Magenta,Black);
		Disphzs24(0xc6e4,0,598,Magenta,Yellow);Disphzs24(0xcbfc,0,622,Magenta,Yellow);//Çå³ý¹ÊÕÏ¼ÇÂ¼
	  dstr=HZMenu;Dispstr(11,2,0,646,Magenta,Black);
	  SPI_Flash_Erase_Sector(136);SPI_Flash_Erase_Sector(137);
    SPI_Flash_Read(buf,0x8c000,12);
		buf[8]=0xff;buf[9]=0xff;buf[10]=0xff;buf[11]=0xff;	
    SPI_Flash_Erase_Sector(140);
	  SPI_Flash_Write_Page(buf,0x8c000,12);	
    tjlzz=0;tjltrue=0;
	 }	
if(cmd==131) //Çå³ýµãÎ»
	 {t=(40+ctime[3])*100+(45+ctime[2]);
		if(wifi==t)
		  {dstr=XCBCMenu;Dispstr(11,2,0,550,Magenta,Black); 
			 Disphzs24(0xb5e3,0,598,Magenta,Black);Disphzs24(0xcebb,0,622,Magenta,Black);
			 llpp=8;counter=3200;
			} 
	 }
//iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
dstr=DJMenu;Dispstr(10,2,0,700,Magenta,Black);Delay16s(10000,10000);	
goto loop2;
	 	
}

 
 
void TXTest(void)
{u16 i,t,rb,cb,k;
 txcsfg=1;//ÖÃÍ¨Ñ¶²âÊÔ±êÖ¾
loop:Disptu(48,Magenta,0,0);
	  Dispsz24(txcsfg,1,0,48,Magenta,Black);
	  if(txcsfg==1)Dispsz24(9600,4,0,184,Magenta,Black);
	  if(txcsfg==2)Dispsz24(1200,4,0,184,Magenta,Black);
	  if(txcsfg==3){Dispsz24(115,3,0,184,Magenta,Black);Dispsz24(200,3,0,232,Magenta,Black);}
	  if(txcsfg==6)Dispsz24(9600,4,0,184,Magenta,Black);
		if(txcsfg==7)Dispsz24(9600,4,0,184,Magenta,Black);
    k=0;dwpack=0;for(i=0;i<16384;i++)downb[i]=0;
	  rb=48;cb=0;
 while(1)
	 {while(dwpack>k){DispszHEX(downb[k],rb,cb,White,Black); 
                    k++;if(k==16384){k=0;dwpack=0;}
                    cb=cb+40;if(cb==800){cb=0;rb=rb+24;}
					          if(rb==480){rb=48;cb=0;ClearS(48,0,800,432,White);
					                      //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷
								               }
					         }
     key=0;t=5000;
     do{KEYdeal();}while((key==0xff)&&(t--)>0);
			if((txcsfg==3)&&(lpfresh>=10))//1000msÑ²¼ì»ØÂ·
           {lpfresh=0;
					  if(lpzz==79)lpzz=0; 
					  Usart_loop(lpzz,strblp[lpzz][5]);
						Dispsz16(strblp[lpzz][5],2,0,750,Magenta,Black); 
					  lpzz++;
					 }
     if(key==0x13){lpfresh=0;txcsfg=0;dwpack=0;for(i=0;i<16384;i++)downb[i]=0; 
                   return;}
     if(key==0x0c){txcsfg++;Delay16s(3000,3000);if(txcsfg>=4)txcsfg=6;goto loop;}
     if(key==0x0b){txcsfg=1;Delay16s(3000,3000);goto loop;}
	  }
}



void XTJL(void)
{u8 i,j,shuz[4];
 u16 rb;
loop2:Disptu(49,Magenta,0,0);
loop:for(i=0;i<4;i++)shuz[i]=0;j=0;rb=100;
	   Dispzf24('_',100,112,Magenta,White);
loop1:key=0;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key==0x13) return; 
			if(key==0xd) goto loop;
      if(key<=3){Dispsz24(key,1,rb,112,Magenta,White);shuz[j]=key;rb=rb+48;Dispzf24('_',rb,112,Magenta,White);j++;
                 if(j==2){if(shuz[1]==1)PrinterJL_date(shuz[0]);
									        else          PrinterJL_num(shuz[0]);
									        goto loop2; 
								         }
                }
	   goto loop1;
}



void PrinterJL_date(u8 jl)
{u8 i,j,x,y,alm,area,dll,daa,nam,flo,zon;//4-year,3-mon,2-day,1-hour,0-min;
 u8 fwbuf[100],shuz[6],buf[32],time[5],t[3];
 u16 k,num,cb;
 u32 waddr[3],addr;
loop5:Disptu(50,Magenta,0,0);
//loop:
	   for(i=0;i<6;i++)shuz[i]=0;j=0;
	   Dispzf24('_',200,200,Magenta,White);cb=200;
loop2:key=0;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key==0x13) return;            
      if(key==0xd) goto loop5;
      if(key==0xc){if(jl==1)
                     {if(hjjltrue>hjjlzz)addr=0x80000+(hjjltrue-hjjlzz)*8;else addr=0x80000;
		                  k=hjjlzz*8;}
									 if(jl==2)
                     {if(ldjltrue>ldjlzz)addr=0x84000+(ldjltrue-ldjlzz)*8;else addr=0x84000;
		                  k=ldjlzz*8;}
                   if(jl==3)
                     {if(tjltrue>tjlzz)addr=0x88000+(tjltrue-tjlzz)*8;else addr=0x88000;
	                    k=tjlzz*8;}
				           goto loop1;}
      if(key<10){Dispsz24(key,1,200,cb,Magenta,White);shuz[j]=key;j++;cb=cb+16;Dispzf24('_',200,cb,Magenta,White);
                 if(j==6){t[0]=shuz[0]*10+shuz[1];t[1]=shuz[2]*10+shuz[3];t[2]=shuz[4]*10+shuz[5];}									
                }
	   goto loop2;
loop1:SPI_Flash_Read(downb,addr,k);
      cb=0;					
for(num=0;num<k;num=num+8)
    {time[4]=downb[num+4];//Äê
	   x=downb[num+5]>>5;y=downb[num+6]>>5;time[3]=x*8+y;//ÔÂ
	   time[2]=downb[num+5]&0x1f;//ÈÕ
     if((t[0]!=time[4])||(t[1]!=time[3])||(t[2]!=time[2]))continue;
		 Disptu(52,Magenta,0,0);Dispsz24((num/8+1),4,400,160,Magenta,White);cb++;Dispsz24(cb,4,400,300,Magenta,White);		
	   for(i=0;i<100;i++)fwbuf[i]=0;
	   for(i=0;i<3;i++)waddr[i]=0;
	   for(i=0;i<5;i++)time[i]=0;
	   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
	   alm=downb[num];dll=downb[num+1];daa=downb[num+2];  
     time[4]=downb[num+4];//Äê
	   x=downb[num+5]>>5;y=downb[num+6]>>5;time[3]=x*8+y;//ÔÂ
	   time[2]=downb[num+5]&0x1f;//ÈÕ
	   time[1]=downb[num+6]&0x1f;//Ê±
	   time[0]=downb[num+7];     //·Ö
//¿ªÊ¼´òÓ¡		
j=0;x=0;		
for(j=0;j<20;j++) {if((j%6)==0)fwbuf[j]=0x2a;else fwbuf[j]=0x20;}fwbuf[j++]=0x0a;//´òÓ¡·Ö¸ô·û(5¸ö*),µ½±ß½ç×Ô¶¯»»ÐÐ
fwbuf[j++]=0x1c;fwbuf[j++]=0x26;
if(alm<=1) waddr[0]=H_alm+8;else waddr[0]=H_alm+alm*8;
if(dll>128||(daa==0&&dll>0)){
                              if(dll<=4)x=65;//»ØÂ·
                              if(dll==129)x=66;//Ö÷µç
                              if(dll==130)x=67;//±¸µç
	                            if(dll==131)x=68;//Éù¹â
                              if(dll==132)x=69;//Â¥ÏÔ
	                            if(dll==134)x=70;//CRT
	                            if(dll==151)x=81;//Áª¶¯·½Ê½:ÊÖ¶¯
	                            if(dll==152)x=82;//Áª¶¯·½Ê½:×Ô¶¯
                              waddr[1]=H_alm+x*8;
	                            SPI_Flash_Read(buf,waddr[0],4);for(i=0;i<4;i++)fwbuf[j++]=buf[i];
                              fwbuf[j++]=0x20;
	                            SPI_Flash_Read(buf,waddr[1],12);//Éè±¸Ãû³Æ
	                            for(i=0;i<4;i++)fwbuf[j++]=buf[i];//ÌØÊâÉè±¸2¸ö×Ö
                              goto loop3;}
 if(dll==0&&daa==0){//Á½¸öºº×Öfwbuf[j++]=0x0a;
                   	SPI_Flash_Read(buf,waddr[0],4);  
                    for(i=0;i<4;i++)fwbuf[j++]=buf[i];
                    goto loop3;}
 if(dll<=16)waddr[2]=0xa0000+(dll-1)*6400+(daa-1)*32;
 if(dll>=41&&dll<=50)waddr[2]=0xd0000+(dll-41)*4096+(daa-1)*32;//×¨ÏßÅÌ
 SPI_Flash_Read(buf,waddr[2],26);
 for(i=6;i<26;i++)fwbuf[j++]=buf[i];								
 nam=buf[0];flo=buf[1];zon=buf[2];	
 fwbuf[j++]=0x0a; //×ßÖ½										
 waddr[1]=H_nam+nam*16;//Éè±¸Ãû³Æ
 SPI_Flash_Read(buf,waddr[0],4);for(i=0;i<4;i++)fwbuf[j++]=buf[i];//±¨¾¯Ãû³Æ
 fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ
 SPI_Flash_Read(buf,waddr[1],12);//Éè±¸Ãû³Æ
 for(i=0;i<8;i++)fwbuf[j++]=buf[i];//Éè±¸Ãû³Æ4¸ö×Ö
 if(zon<128&&zon>0){fwbuf[j++]=zon/10+0x30;fwbuf[j++]=zon%10+0x30;fwbuf[j++]=0xc7;fwbuf[j++]=0xf8;}//·ÖÇø
 if(flo>0&&flo<0xff){if(flo>128){flo=flo-128;fwbuf[j++]=0x2d;}//Â¥²ã
	                   fwbuf[j++]=flo/10+0x30;fwbuf[j++]=flo%10+0x30;fwbuf[j++]=0x46;//F //fwbuf[j++]=0xb2;fwbuf[j++]=0xe3;//²ã
	                  }
  //´òÓ¡ÄÚÈÝËÍ»º³åÇø
loop3:fwbuf[j++]=0x0a;//×ßÖ½						
	fwbuf[j++]=0x4d;fwbuf[j++]=0x3d;if(area<=9){fwbuf[j++]=0x30+area/10;fwbuf[j++]=0x30+area%10;}else fwbuf[j++]=0x30;fwbuf[j++]=0x20;//»ØÂ·ºÅ										
  fwbuf[j++]=0x4c;fwbuf[j++]=0x3d;if(dll<=8){fwbuf[j++]=0x30+dll/10;fwbuf[j++]=0x30+dll%10;}else fwbuf[j++]=0x30;fwbuf[j++]=0x20;
  fwbuf[j++]=0x41;fwbuf[j++]=0x3d;fwbuf[j++]=daa/100+0x30;fwbuf[j++]=(daa%100)/10+0x30;fwbuf[j++]=daa%10+0x30;fwbuf[j++]=0x20;//µØÖ·
  fwbuf[j++]=time[4]/10+0x30;fwbuf[j++]=time[4]%10+0x30;fwbuf[j++]=0x2f;//Äê/ÔÂ/ÈÕ
  fwbuf[j++]=time[3]/10+0x30;fwbuf[j++]=time[3]%10+0x30;fwbuf[j++]=0x2f;
  fwbuf[j++]=time[2]/10+0x30;fwbuf[j++]=time[2]%10+0x30;fwbuf[j++]=0x20;
  fwbuf[j++]=time[1]/10+0x30;fwbuf[j++]=time[1]%10+0x30;fwbuf[j++]=0x3a;
  fwbuf[j++]=time[0]/10+0x30;fwbuf[j++]=time[0]%10+0x30;					
  fwbuf[j++]=0x0d;																
  for(i=0;i<j;i++)  Printing(fwbuf[i]);
   }	
 dstr=DJMenu;Dispstr(10,2,400,400,Magenta,Black);		
 goto loop5;
}




void PrinterJL_num(u8 jl)
{u8 i,j,x,y,alm,area,dll,daa,nam,flo,zon;
 u8 fwbuf[100],shuz[8],time[5],buf[32];
 u16 k,num,cb,t[2];
 u32 waddr[3],addr;
loop5:Disptu(51,Magenta,0,0);
loop:for(i=0;i<8;i++)shuz[i]=0;j=0;
	   Dispzf24('_',200,120,Magenta,White);cb=120;
loop2:key=0;Delay16s(3000,3000);
      do{KEYdeal();}while(key==0xff);
      if(key==0x13) return;            
      if(key==0xd) goto loop;
      if(key==0xc)
			 {if(jl==1)
          {if(hjjltrue>hjjlzz)addr=0x80000+(hjjltrue-hjjlzz)*8+(t[0]-1)*8;else addr=0x80000+(t[0]-1)*8;
		       k=(t[1]-t[0]+1)*8;}
				if(jl==2)
          {if(ldjltrue>ldjlzz)addr=0x84000+(ldjltrue-ldjlzz)*8+(t[0]-1)*8;else addr=0x84000+(t[0]-1)*8;
		       k=(t[1]-t[0]+1)*8;}
        if(jl==3)
          {if(tjltrue>tjlzz)addr=0x88000+(tjltrue-tjlzz)*8+(t[0]-1)*8;else addr=0x88000+(t[0]-1)*8;
	         k=(t[1]-t[0]+1)*8;}
			  goto loop1;
			 }
      if(key<10){Dispsz24(key,1,200,cb,Magenta,White);shuz[j]=key;cb=cb+16;j++; 
				        if(j==4)cb=216;Dispzf24('_',200,cb,Magenta,White);
                if(j==8){t[0]=shuz[0]*1000+shuz[1]*100+shuz[2]*10+shuz[3];t[1]=shuz[4]*1000+shuz[5]*100+shuz[6]*10+shuz[7];
								        if(t[1]<t[0])goto loop;}
                }
	   goto loop2;
loop1:SPI_Flash_Read(downb,addr,k); 
for(num=0;num<k;num=num+8)
    {Disptu(52,Magenta,0,0);Dispsz24((num/8+1),4,400,160,Magenta,White);
		 for(i=0;i<100;i++)fwbuf[i]=0;
	   for(i=0;i<3;i++)waddr[i]=0;
	   for(i=0;i<5;i++)time[i]=0;
	   //iwdgtim=0;IWDG_ReloadCounter();//ÖØÐÂ×°ÔØ¿´ÃÅ¹·¼ÆÊýÆ÷ 
	   alm=downb[num];dll=downb[num+1];daa=downb[num+2];  
     time[4]=downb[num+4];//Äê
	   x=downb[num+5]>>5;y=downb[num+6]>>5;time[3]=x*8+y;//ÔÂ
	   time[2]=downb[num+5]&0x1f;//ÈÕ
	   time[1]=downb[num+6]&0x1f;//Ê±
	   time[0]=downb[num+7];     //·Ö
//¿ªÊ¼´òÓ¡
j=0;x=0;		
for(j=0;j<20;j++) {if((j%6)==0)fwbuf[j]=0x2a;else fwbuf[j]=0x20;}fwbuf[j++]=0x0a;//´òÓ¡·Ö¸ô·û(5¸ö*),µ½±ß½ç×Ô¶¯»»ÐÐ
fwbuf[j++]=0x1c;fwbuf[j++]=0x26;
if(alm<=1) waddr[0]=H_alm+8;else waddr[0]=H_alm+alm*8;
if(dll>128||(daa==0&&dll>0)){//ËÄ¸öºº×Ö
                              if(dll<=4)x=65;//»ØÂ·
                              if(dll==129)x=66;//Ö÷µç
                              if(dll==130)x=67;//±¸µç
	                            if(dll==131)x=68;//Éù¹â
                              if(dll==132)x=69;//Â¥ÏÔ
	                            if(dll==134)x=70;//CRT
	                            if(dll==151)x=81;//Áª¶¯·½Ê½:ÊÖ¶¯
	                            if(dll==152)x=82;//Áª¶¯·½Ê½:×Ô¶¯
                              waddr[1]=H_alm+x*8;
	                            SPI_Flash_Read(buf,waddr[0],4);for(i=0;i<4;i++)fwbuf[j++]=buf[i];
                              fwbuf[j++]=0x20;
	                            SPI_Flash_Read(buf,waddr[1],12);//Éè±¸Ãû³Æ
	                            for(i=0;i<4;i++)fwbuf[j++]=buf[i];//ÌØÊâÉè±¸2¸ö×Ö
                              goto loop3;}
 if(dll==0&&daa==0){//Á½¸öºº×Öfwbuf[j++]=0x0a;
                   	SPI_Flash_Read(buf,waddr[0],4);  
                    for(i=0;i<4;i++)fwbuf[j++]=buf[i];
                    goto loop3;}
 if(dll<=16)waddr[2]=0xa0000+(dll-1)*6400+(daa-1)*32;
 if(dll>=41&&dll<=50)waddr[2]=0xd0000+(dll-41)*4096+(daa-1)*32;//×¨ÏßÅÌ									
 SPI_Flash_Read(buf,waddr[2],26);
 for(i=6;i<26;i++)fwbuf[j++]=buf[i];								
 nam=buf[0];flo=buf[1];zon=buf[2];	
 fwbuf[j++]=0x0a; //×ßÖ½										
 waddr[1]=H_nam+nam*16;
 SPI_Flash_Read(buf,waddr[0],4);for(i=0;i<4;i++)fwbuf[j++]=buf[i];
 fwbuf[j++]=0x20;//´òÓ¡¿Õ¸ñ
 SPI_Flash_Read(buf,waddr[1],12);
 for(i=0;i<12;i++)fwbuf[j++]=buf[i];
 if(zon<128&&zon>0){fwbuf[j++]=zon/10+0x30;fwbuf[j++]=zon%10+0x30;fwbuf[j++]=0xc7;fwbuf[j++]=0xf8;}//·ÖÇø
 if(flo>0&&flo<0xff){if(flo>128){flo=flo-128;fwbuf[j++]=0x2d;}//Â¥²ã
	                   fwbuf[j++]=flo/10+0x30;fwbuf[j++]=flo%10+0x30;fwbuf[j++]=0x46;//F //fwbuf[j++]=0xb2;fwbuf[j++]=0xe3;//²ã
	                  }
  //´òÓ¡ÄÚÈÝËÍ»º³åÇø
loop3:fwbuf[j++]=0x0a;//×ßÖ½									
	fwbuf[j++]=0x4d;fwbuf[j++]=0x3d;if(area<=9){fwbuf[j++]=0x30+area/10;fwbuf[j++]=0x30+area%10;}else fwbuf[j++]=0x30;fwbuf[j++]=0x20;										
  fwbuf[j++]=0x4c;fwbuf[j++]=0x3d;if(dll<=8){fwbuf[j++]=0x30+dll/10;fwbuf[j++]=0x30+dll%10;}else fwbuf[j++]=0x30;fwbuf[j++]=0x20;//»ØÂ·
  fwbuf[j++]=0x41;fwbuf[j++]=0x3d;fwbuf[j++]=daa/100+0x30;fwbuf[j++]=(daa%100)/10+0x30;fwbuf[j++]=daa%10+0x30;fwbuf[j++]=0x20;//µØÖ·
  fwbuf[j++]=time[4]/10+0x30;fwbuf[j++]=time[4]%10+0x30;fwbuf[j++]=0x2f;//Äê/ÔÂ/ÈÕ
  fwbuf[j++]=time[3]/10+0x30;fwbuf[j++]=time[3]%10+0x30;fwbuf[j++]=0x2f;
  fwbuf[j++]=time[2]/10+0x30;fwbuf[j++]=time[2]%10+0x30;fwbuf[j++]=0x20;
  fwbuf[j++]=time[1]/10+0x30;fwbuf[j++]=time[1]%10+0x30;fwbuf[j++]=0x3a;//Ê±:·Ö
  fwbuf[j++]=time[0]/10+0x30;fwbuf[j++]=time[0]%10+0x30;						
  fwbuf[j++]=0x0d;											
  for(i=0;i<j;i++)  Printing(fwbuf[i]);	
   }
dstr=DJMenu;Dispstr(10,2,400,400,Magenta,Black); 
goto loop5;
}



void Download_PC(void)
{u16 m;  //curv,
	u8 i,j;  //dll,
//	u8 buf[10];
	u32 waddr,wadrc,k=0;
	Disphzs(0xcfc2,TiShi_y,TiShi_x,Blue,White);
	Disphzs(0xd4d8,TiShi_y,TiShi_x+16,Blue,White);
//USART1_SendByte(0xf0);
	do{i=dwlong/10000;m=dwlong%10000;
		Dispsz16(i,1,TiShi_y,TiShi_x+34,Blue,White);
		Dispsz16(m,4,TiShi_y,TiShi_x+42,Blue,White);
	}while(DownL==2);//µÈ´ý½ÓÊÕ½áÊø 
//USART1_SendByte(downb[1]);
	dstr=XCMenu;
	if(downb[1]==1)//ÏÔÊ¾±à³Ì_a0000_ac800_b9000_bf400
	{	Dispstr16(1,4,TiShi_y+17,TiShi_x,Blue,White);
   	for(k=6;k<0xc806;k=k+32)  //yuan 8198
		{	for(j=6;j<26;j++)
			{	i=downb[k+j];if(i<0xa0&&i>0x9a)downb[k+j]=downb[k+j]+0x60;}
		}
		if(downb[2]==1)
		{	waddr=0xa0000+0xc800;
			SPI_Flash_Read(downb+0xd006,waddr,0x800); //9loop_UPhalf read
			for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(160+j);}
			waddr=0xa0000;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=0xa0000+0xc800;
			SPI_Flash_Write_NoCheck(downb+0xd006,waddr,0x800);
		}
		if(downb[2]==2)
		{	waddr=0xa0000+0xc000;
			SPI_Flash_Read(downb+0xd006,waddr,0x800); //8loop_Downhalf read
			for(j=12;j<=26;j++)	{ SPI_Flash_Erase_Sector(160+j);}
			waddr=0xa0000+0xc800;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=0xa0000+0xc000;
			SPI_Flash_Write_NoCheck(downb+0xd006,waddr,0x800);
		}
		if(downb[2]==3)
		{	waddr=0xa0000+0x19000;
			for(j=27;j<=34;j++)	{ SPI_Flash_Erase_Sector(160+j);}
			SPI_Flash_Write_NoCheck(downb+6,waddr,0x6400);
		}
		if(downb[2]==4)
		{	wadrc=0x100000;
			waddr=wadrc+0xc800;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //9loop_half read
			for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(256+j);}
			waddr=wadrc;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD4);
		}
		if(downb[2]==5)
		{	wadrc=0x100000;
			waddr=wadrc+0xc000;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //8loop_half read
			for(j=13;j<=26;j++)	{ SPI_Flash_Erase_Sector(256+j);}
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc000;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD5);
		}
		if(downb[2]==6)
		{	wadrc=0x120000;
			waddr=wadrc+0xc800;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //9loop_half read
			for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(288+j);}
			waddr=wadrc;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD6);
		}
		if(downb[2]==7)
		{	wadrc=0x120000;
			waddr=wadrc+0xc000;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //8loop_half read
			for(j=13;j<=26;j++)	{ SPI_Flash_Erase_Sector(288+j);}
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc000;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD7);
		}
		if(downb[2]==8)
		{	wadrc=0x140000;
			waddr=wadrc+0xc800;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //9loop_half read
			for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(320+j);}
			waddr=wadrc;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD8);
		}
		if(downb[2]==9)
		{	wadrc=0x140000;
			waddr=wadrc+0xc000;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //8loop_half read
			for(j=13;j<=26;j++)	{ SPI_Flash_Erase_Sector(320+j);}
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc000;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xD9);
		}
		if(downb[2]==10)
		{	wadrc=0x160000;
			waddr=wadrc+0xc800;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //9loop_half read
			for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(352+j);}
			waddr=wadrc;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xDA);
		}
		if(downb[2]==11)
		{	wadrc=0x160000;
			waddr=wadrc+0xc000;
			SPI_Flash_Read(downb+0xd000,waddr,0x800); //8loop_half read
			for(j=13;j<=26;j++)	{ SPI_Flash_Erase_Sector(352+j);}
			waddr=wadrc+0xc800;
			SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
			waddr=wadrc+0xc000;
			SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
USART1_SendByte(0xDB);
		}
		
//	USART1_SendByte(0xf1);
	}
	if(downb[1]==2)//Áª¶¯±à³Ì_e0000_e8000,ef000_ef800
	{	Dispstr16(10,4,TiShi_y+17,TiShi_x,Blue,White);
		SPI_Flash_Erase_Block(14);
		SPI_Flash_Write_NoCheck(downb+6,0xef000,0x800);
//USART1_SendByte(midlong&0x000000ff);Delay16s(20,20);USART1_SendByte((midlong>>8)&0x000000ff);Delay16s(20,20);
		SPI_Flash_Write_NoCheck(downb+6+0x800,0xe0000,(midlong+0x100));
//waddr=0xe0000;
//SPI_Flash_Read(downb,waddr,midlong); //9loop_half read
//for(k=0;k<midlong;k++)  
//{	USART1_SendByte(downb[k]);Delay16s(20,20);
//}
USART1_SendByte(0xff);
	}
	if(downb[1]==3)//×ÜÏßÅÌ±à³Ì_9c000_9d000 2x256x8=4096=1000h
	{	Dispstr16(36,4,TiShi_y+17,TiShi_x,Blue,White);
		SPI_Flash_Erase_Sector(156);
		SPI_Flash_Write_NoCheck(downb+6,0x9c000,0x1000);
//USART1_SendByte(0xf3);
	} 
	if(downb[1]==7)//¹ã²¥Éù¹â±à³Ì_9d000_9e000 2x256x8=4096=1000h
	{	Dispstr16(46,4,TiShi_y+17,TiShi_x,Blue,White);
		SPI_Flash_Erase_Sector(157);
		SPI_Flash_Write_NoCheck(downb+6,0x9d000,0x1000);
//USART1_SendByte(0xf7);
	} 
	if(downb[1]==4)//ÇøÓòÃû±à³Ì_9e000_9e600 6x256=1536=600h 1B
	{	
		for(m=6;m<(6+6*256);m++)
			{	i=downb[m];if((i<0xa0)&&(i>0x9a))downb[m]=downb[m]+0x60;}
		Dispstr16(64,4,TiShi_y+17,TiShi_x,Blue,White);
		SPI_Flash_Erase_Sector(158);
		SPI_Flash_Write_NoCheck(downb+6,0x9e000,0x600);
USART1_SendByte(0xbb);
	}
	if(downb[1]==5)//Éè±¸Ãû³Æ_9f000_9ffff
	{	Dispstr16(19,4,TiShi_y+17,TiShi_x,Blue,White);
		for(k=6;k<0x806;k++)
		{i=downb[k];if(i<0xa0&&i>0x9a)downb[k]=downb[k]+0x60;}
		SPI_Flash_Erase_Sector(159);
		SPI_Flash_Write_NoCheck(downb+6,0x9f000,0x800);
//USART1_SendByte(0xf5);
	}
	if(downb[1]==9)//  ÏÂÔØ×Ö¿â_0~6 00000_6ffff
	{	Dispstr16(1,4,TiShi_y+17,TiShi_x,Blue,White);
//   	for(k=6;k<0xc806;k=k+32)  //yuan 8198
//		{	for(j=6;j<26;j++)
//			{	i=downb[k+j];if(i<0xa0&&i>0x9a)downb[k+j]=downb[k+j]+0x60;}
//		}
		SPI_Flash_Erase_Block(downb[2]);
		waddr=downb[2]*0x10000;
		SPI_Flash_Write_NoCheck(downb+6,waddr,0xffff);
//	USART1_SendByte(0xf9);
	}
	
	midlong=0;DownL=2;
//USART1_SendByte(0xf9);
	dstr=FLMenu; Dispstr16(6,4,TiShi_y+17,TiShi_x+64,Blue,White);
	return;
}
 

void Upload_PC(u8 type)
{u16 k=0,t,zz,num; //curv,
 u8 i,j,m;         //dll,
 u32 waddr;
 	Disphzs(0xc9cf,TiShi_y,TiShi_x,Blue,White);
	Disphzs(0xb4ab,TiShi_y,TiShi_x+16,Blue,White);
//USART1_SendByte(0xf1);
 dstr=XCMenu;
 if(type==1)//ÏÔÊ¾±à³Ì
   {Dispstr(1,4,456,0,White,Magenta);
		for(m=0;m<8;m++)
		   {downb[0]=0xfd;downb[1]=1;downb[2]=m+1;downb[3]=0x4c;downb[4]=0x51;downb[5]=0x5c;
        waddr=0xa0000+m*6400;				 
        SPI_Flash_Read(downb+6,waddr,8192);
   	    for(k=6;k<8198;k=k+32){for(j=6;j<26;j++){i=downb[k+j];if(i==0xfd||i==0xfe)downb[k+j]=downb[k+j]-0x60;}}
   // SPI_Flash_Read(downb+16390,0x76800,512);	
		//for(k=16390;k<16902;k=k+32){for(j=6;j<26;j++){i=downb[k+j];if(i==0xfd||i==0xfe)downb[k+j]=downb[k+j]-0x60;}}	
	      downb[8198]=0xfe;
        for(k=0;k<8198;k++)//´«ÊäÎÄ¼þ	
           {USART1_SendByte(downb[k]);Dispsz24(k,4,456,116,White,Magenta);} 
			 }
   }
 if(type==2)//Áª¶¯±à³Ì
   {Dispstr(10,2,456,0,White,Magenta);
		downb[0]=0xfd;downb[1]=2;downb[2]=0;downb[3]=0x4c;downb[4]=0x0a;downb[5]=0x18;
		SPI_Flash_Read(downb+6,0xef000,1024);
		downb[1030]=0xfe;
		for(k=0;k<1031;k++){USART1_SendByte(downb[k]);Dispsz24(k,4,456,100,White,Magenta);}//´«ÊäÎÄ¼þ
		Delay16s(20000,20000);
		//Áª¶¯±à³ÌÊý¾Ý°ü
		dstr=XCMenu;Dispstr(10,4,456,300,White,Magenta); 
		num=(downb[6+1022]<<8)+downb[6+1023];
		zz=(downb[6+num*2]<<8)+downb[7+num*2];
		downb[0]=0xfd;downb[1]=2;downb[2]=1;downb[3]=0x4c;downb[4]=num/100;downb[5]=num%100;
		SPI_Flash_Read(downb+6,0xe0000,zz);
		downb[6+zz]=0xfe;
		for(k=0;k<(zz+7);k++)
		   {USART1_SendByte(downb[k]);
			  if(k<10000){Dispsz24(0,1,456,400,White,Magenta);Dispsz24(k,4,456,416,White,Magenta);}
				else{t=k-10000;Dispsz24(1,1,456,400,White,Magenta);Dispsz24(t,4,456,416,White,Magenta);} 
			 }
   }
 if(type==5)//Éè±¸Ãû³Æ
   {Dispstr(19,4,456,0,White,Magenta);
		downb[0]=0xfd;downb[1]=3;downb[2]=0;downb[3]=0x4c;downb[4]=0x14;downb[5]=0x30;
    SPI_Flash_Read(downb+6,0x9f000,2048);
		for(k=6;k<2054;k++){i=downb[k];if(i==0xfd||i==0xfe)downb[k]=downb[k]-0x60;}
		downb[2054]=0xfe;
    for(k=0;k<2055;k++){USART1_SendByte(downb[k]);Dispsz24(k,4,456,100,White,Magenta);}
   }
if(type==9)//²ãÏÔ±à³Ì
	{	
		Disphzs(0xb2e3,TiShi_y,TiShi_x+32,Blue,White);
		Disphzs(0xcfd4,TiShi_y,TiShi_x+48,Blue,White);

/*	downb[0]=0xfd;downb[1]=1;downb[2]=0x3e;downb[3]=0x4c;downb[4]=0x06;downb[5]=0x00; //quyu
		SPI_Flash_Read(downb+6,0x9e000,0x600);
	  downb[0x606]=0xfe;
    for(k=0;k<=0x606;k++){USART1_SendByte(downb[k]);}	
		i=4; Dispsz16(i,2,TiShi_y,TiShi_x+64,Blue,White);
		Delay1ms(8000);  //4s   8000
*/
		downb[0]=0xfd;downb[1]=1;downb[2]=0x3f;downb[3]=0x4c;downb[4]=0x08;downb[5]=0x00; //SheBei
		SPI_Flash_Read(downb+6,0x9f000,0x800);
	  downb[0x806]=0xfe;
    for(k=0;k<=0x806;k++){USART1_SendByte(downb[k]);}	
		i=5; Dispsz16(i,2,TiShi_y,TiShi_x+64,Blue,White);
		Delay1ms(8000);  //4s   8000

		downb[0]=0xfd;downb[1]=1;downb[3]=0x4c;downb[4]=0x1f;downb[5]=0x40; //Room
		downb[0x1f46]=0xfe;
		for(j=0;j<16;j++)
		{	
			downb[2]=0x40+j; downb[6406]=0xfe;
			SPI_Flash_Read(downb+6,0xa0000+j*6400,6400);
			for(k=0;k<6406;k++){USART1_SendByte(downb[k]);}	
			for(k=0;k<1600;k++){USART1_SendByte(0xff);}	
			USART1_SendByte(downb[6406]);
			Dispsz16(j+1,3,TiShi_y,TiShi_x+64,Blue,White);
			Delay1ms(8000);  //4s   8000
		}
//		do{i=dwlong/10000;m=dwlong%10000;
//		Dispsz16(i,1,TiShi_y,TiShi_x+34,Blue,White);
//		Dispsz16(m,4,TiShi_y,TiShi_x+42,Blue,White);
//	}while(DownL==2);	//wait TX end 

   }
if(type==3)//×ÜÏß±à³Ì
   {Dispstr(37,4,456,0,White,Magenta);
		downb[0]=0xfd;downb[1]=5;downb[2]=0;downb[3]=0x4c;downb[4]=0x28;downb[5]=0x60;
		SPI_Flash_Read(downb+6,0x9c000,4096);
	  downb[4102]=0xfe;
    for(k=0;k<4103;k++){USART1_SendByte(downb[k]);Dispsz24(k,4,456,100,White,Magenta);}	
   }	 
 dstr=DJMenu;Dispstr(10,2,456,600,White,Black);Delay16s(1000,2000);
 return;
}
	




void Set_point(void)
{u8 i,j,cb,shuz[3],buf[8];
 Disptu(14,Magenta,0,0);
loop1:ClearXY(64,112,24,400,Magenta);Dispzf24('_',64,112,Magenta,White);
      for(i=0;i<3;i++)shuz[i]=0;j=0;cb=112;
loop:key=0;Delay16s(3000,3000);
     do{KEYdeal();}while(key==0xff);
     if(key==0x0d)goto loop1;
		 if(key==0x13)return;
     if(key<8)
			  {shuz[j]=key;j++;Dispsz24(key,1,64,cb,Magenta,White);
			   cb=cb+16;Dispzf24('_',64,cb,Magenta,Black);
			   if(j==2){if(shuz[1]==1&&shuz[0]<9&&shuz[0]>0)
					           {Dispzf24('O',64,128,Magenta,Black);Dispzf24('K',64,144,Magenta,Black);
											llpp=shuz[0];counter=shuz[0]*64;
					            buf[0]=counter>>8;buf[1]=counter&0x00ff;buf[2]=llpp;
						          buf[3]=0xff;buf[4]=0xff;buf[5]=ctime[4];buf[6]=ctime[4];buf[7]=ctime[2];
						          //Ð´µãÎ»
                      SPI_Flash_Erase_Sector(110);
				              SPI_Flash_Write_NoCheck(buf,0x6e000,8);//Ð´Èë25x40
                     }
					       else goto loop1;
   					    }								 
			 }	  
    goto loop;
}
void Clear_Sdram_Lcd(void)
{
	u32 z;
	/* ½ûÖ¹Ð´±£»¤ */
  FMC_SDRAMWriteProtectionConfig(FMC_Bank2_SDRAM, DISABLE); 
  while(FMC_GetFlagStatus(FMC_Bank2_SDRAM, FMC_FLAG_Busy) != RESET){} 
  for(z=0;z<768000;z=z+2)
	   { 
			 *( uint32_t*)(SDRAM_BANK_ADDR+z)=(uint16_t) 0;
		 }
}


void JianCha(void){					//¼ì²é			
	//	USART1_SendByte(0xbb);
	chaType=4; //ÍË³ö±êÊ¶
	Checkpg = 0;
	JianChaShow();
	//ÐÂÔö
	ClearArray();		//Êý×éÇåÁã
	CheckCntStat();
	DispCheck_JLB(Checkpg);  //Ä¬ÈÏÏÔÊ¾µÚ0Ò³
}
//ÐÂÔö
void ClearArray(void){
	u8 i;
	for(i=0;i<NormalWorkCntArray_n;i++){
		NormalWorkCntArray[i]=ShieldCntArray[i]=BreakdownCntArray[i]=0;
	}
}
void CheckCntStat(void){
	u8 loop,addr;
	//µÇ¼Ç±í²éÑ¯
	for(loop=0;loop<16;loop++){
		for(addr=1;addr<=200;addr++){
			//Õý³£¹¤×÷Êý
			if((djb[loop][addr]&0xc4)==0x80){
				NormalWorkStatInc(loop,addr);
			}
			//¹ÊÕÏ
			else if((djb[loop][addr]&0xc4)==0x84){
				BreakDownStatInc(loop,addr);
			}
			//ÆÁ±Î
			else if((djb[loop][addr]&0xc4)==0xc0){
				ShieldStatInc(loop,addr);
			}
		}
	}
}
void JianChaShow(void){
	ClearXY(LDtop_y,type_x,260,670,Black);						//µ×É«
	LCD_DrawSquare(left_x-1,LDtop_y-1,righ_x,LDlow_y,Yellow);					// ¼ì²é0(¹¦ÄÜ+ÐÐÊý)
	LCD_DrawSquare(left_x-1,LDtop_y-1+27,righ_x,LDlow_y,Yellow);			// ¼ì²é1
	LCD_DrawSquare(left_x-1,LDtop_y-1+54,righ_x,LDlow_y,Yellow);			// ¼ì²é2
	LCD_DrawSquare(left_x-1,LDtop_y-1+81,righ_x,LDlow_y,Yellow);			// ¼ì²é3
	LCD_DrawSquare(left_x-1,LDtop_y-1+108,righ_x,LDlow_y,Yellow);			// ¼ì²é4
	LCD_DrawSquare(left_x-1,LDtop_y-1+135,righ_x,GZlow_y,Yellow);			// ¼ì²é5
	LCD_DrawSquare(left_x-1,LDtop_y-1+162,righ_x,GZlow_y,Yellow);			// ¼ì²é6
	LCD_DrawSquare(left_x-1,LDtop_y-1+189,righ_x,GZlow_y,Yellow);			// ¼ì²é7
	LCD_DrawSquare(left_x-1,LDtop_y-1+216,righ_x,GZlow_y,Yellow);			// ¼ì²é8
	LCD_DrawSquare(left_x-1,LDtop_y-1+243,righ_x,GZlow_y,Yellow);			// ¼ì²é9
//¼ì²éÍ·
	dstr=CheckMenu;Dispstr(0,2,LDtop_y,type_x,Black,Smoke);						//ÀàÐÍ
	dstr=CheckMenu;Dispstr(4,4,LDtop_y,DesignCnt_x,Black,Smoke);				//Éè¼ÆÊýÁ¿
	dstr=CheckMenu;Dispstr(12,5,LDtop_y,NormalCnt_x,Black,Smoke); 			//Õý³£¹¤×÷Êý
	dstr=CheckMenu;Dispstr(22,3,LDtop_y,BreakdownCnt_x,Black,Smoke); 	//¹ÊÕÏÊý
	dstr=CheckMenu;Dispstr(28,3,LDtop_y,ShieldCnt_x,Black,Smoke); 			//ÆÁ±ÎÊý
//×ó²àÏÔÊ¾
	Disphzs24(0xc1aa,204,0,Blue,Blue);	  		//Áª(È¥µô)
	Disphzs24(0xbcec,240,0,Blue,White);	  			//¼ì 
	Disphzs24(0xb2e9,324,0,Blue,White);	  			//²é
	Disphzs24(0xd5cf,348,0,Blue,Blue);	  		//ÕÏ(È¥µô)
	Disphzs(0xc6c1,400,0,Blue,Blue);	  			//ÆÁ(È¥µô)
	Disphzs(0xb1ce,418,0,Blue,Blue);	  			//±Î(È¥µô)
	Dispsys();
	DispFactor();
	DispTimeUP();	 
}
/*ÐÂÔö½áÊø*/
void LCD_SetLayer(uint32_t Layerx)
{
  if (Layerx == LCD_BACKGROUND_LAYER)
    {CurrentFrameBuffer = LCD_FRAME_BUFFER; 
     CurrentLayer = LCD_BACKGROUND_LAYER;
    }
  else{
       CurrentFrameBuffer = LCD_FRAME_BUFFER + BUFFER_OFFSET;
       CurrentLayer = LCD_FOREGROUND_LAYER;
      }
}  

void LCD_SetColors(uint16_t TextColor, uint16_t BackColor) 
{
  CurrentTextColor = TextColor; 
  CurrentBackColor = BackColor;
}


void LCD_GetColors(uint16_t *TextColor, uint16_t *BackColor)
{
  *TextColor = CurrentTextColor;
  *BackColor = CurrentBackColor;
}


void LCD_SetTextColor(uint16_t Color)
{
  CurrentTextColor = Color;
}

void LCD_SetBackColor(uint16_t Color)
{
  CurrentBackColor = Color;
}

void LCD_SetTransparency(uint8_t transparency)
{
  if (CurrentLayer == LCD_BACKGROUND_LAYER)   LTDC_LayerAlpha(LTDC_Layer1, transparency);
  else                                        LTDC_LayerAlpha(LTDC_Layer2, transparency);
  LTDC_ReloadConfig(LTDC_IMReload);
}

void ClearS(u16 curv, u16 curh, u16 width, u16 height, u16 Color)
{
   
 DMA2D_InitTypeDef      DMA2D_InitStruct;
 uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
 uint32_t pos = 0;

 Red_Value = (0xF800 & Color) >> 11;
 Blue_Value = 0x001F & Color;
 Green_Value = (0x07E0 & Color) >> 5;
 pos = CurrentFrameBuffer + 2*LCD_PIXEL_WIDTH*curv ;
 /* configure DMA2D */
 DMA2D_DeInit();
 DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;
 DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;
 //ÅäÖÃDMA2DµÄ¼Ä´æÆ÷ÑÕÉ«Öµ
 DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;   
 DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;   
 DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;   
 DMA2D_InitStruct.DMA2D_OutputAlpha = (Color&0x8000) ? 0xFF:0x00;	
 DMA2D_InitStruct.DMA2D_OutputMemoryAdd = pos;
 DMA2D_InitStruct.DMA2D_OutputOffset = 0;
 DMA2D_InitStruct.DMA2D_NumberOfLine = height;//480ÐÐÊý¾Ý
 DMA2D_InitStruct.DMA2D_PixelPerLine = width;//Ã¿ÐÐÓÐ800¸öÏñËØµã
 DMA2D_Init(&DMA2D_InitStruct);
 DMA2D_StartTransfer();
 while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET){} 
 LCD_SetTextColor(Color);
}


void LCD_DrawLine(u16 Xpos,u16 Ypos,u16 Length,u8 Direction,u16 fColor)
{
 DMA2D_InitTypeDef      DMA2D_InitStruct;

 uint32_t  Xaddress = 0;
 uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;

 Xaddress = CurrentFrameBuffer + 2*(LCD_PIXEL_WIDTH*Ypos + Xpos);
 Red_Value = (0xF800 & fColor) >> 11;
 Blue_Value = 0x001F & fColor;
 Green_Value = (0x07E0 & fColor) >> 5;
 /* Configure DMA2D */
 DMA2D_DeInit();
 DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;
 DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;
 //DMA2D_InitStruct.DMA2D_CMode =DMA2D_ARGB1555
 DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;
 DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;
 DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;
 DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;
 DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Xaddress;

 if(Direction == LCD_DIR_HORIZONTAL)
     {
      DMA2D_InitStruct.DMA2D_OutputOffset = 0;
      DMA2D_InitStruct.DMA2D_NumberOfLine = 1;
      DMA2D_InitStruct.DMA2D_PixelPerLine = Length;
     }
 else{
      DMA2D_InitStruct.DMA2D_OutputOffset = LCD_PIXEL_WIDTH - 1;
      DMA2D_InitStruct.DMA2D_NumberOfLine = Length;
      DMA2D_InitStruct.DMA2D_PixelPerLine = 1;
     }
 DMA2D_Init(&DMA2D_InitStruct);	 
 DMA2D_StartTransfer();//¿ªÊ¼DMA2D´«Êä 
 while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET){}   
}


void LCD_DrawSquare(u16 x, u16 y,u16 width,u16 high,u16 color)
{
  LCD_DrawLine(x,y,width,LCD_DIR_HORIZONTAL,color);
	LCD_DrawLine(x,y+high,width,LCD_DIR_HORIZONTAL,color);
	LCD_DrawLine(x,y,high,LCD_DIR_VERTICAL,color);
	LCD_DrawLine(x+width,y,high,LCD_DIR_VERTICAL,color);
}
void LCD_DrawSquareBox(u16 x, u16 y,u16 width,u16 high,u16 color)
{	u16 i;
	for(i=0;i<high;i++){
		LCD_DrawLine(x,y+i,width,LCD_DIR_HORIZONTAL,color);
	}
//	LCD_DrawLine(x,y+high,width,LCD_DIR_HORIZONTAL,color);
//	LCD_DrawLine(x,y,high,LCD_DIR_VERTICAL,color);
//	LCD_DrawLine(x+width,y,high,LCD_DIR_VERTICAL,color);
}



void ClearXY(u16 curv, u16 curh, u16 height, u16 weight, u16 Color)
{
 u32 xpos =0, ypos = 0; //,pos
 u16 i,j;

 xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
 ypos = curh+ypos ;	
 for(i=0; i<height; i++)       
    {
     for(j=0;j<weight; j++)    
        { /* Write data value to SDRAM memory */
          *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = Color;
         ypos++;
        }
    ypos += 800-weight;
   }	
}


void Disphzs(u16 ccode,u16 curv,u16 curh,u16 bColor,u16 fColor)
{ 
  u32 xpos =0, ypos = 0,temp,pos; //,Paddr,Baddr
  u8  i,j,fsz[32];

  xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
  ypos = ypos + curh;
	
  if(ccode>0xd7ff)return;
  if(ccode>0x8000)
     {pos=(((ccode>>8)-0xb0)*94+(ccode&0x00ff)-0xa1)*32;
	    SPI_Flash_Read(fsz,pos,32);	
      for(i=0;i<16;i++)	  
         {temp=(fsz[2*i]<<8)+fsz[2*i+1]; //×ó8ÁÐ+ÓÒ8ÁÐ
          for(j=0;j<16;j++)
					   {temp <<= 1;
						  if((temp&0x10000)!=0) 
					      { /* Write data value to SDRAM memory */
                 *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
                }
							else                 
               { /* Write data value to SDRAM memory */
                *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
               }
             ypos++;
						}
          ypos += 784;
         }
     }
   else{Dispzf((ccode>>8),curv,curh,bColor,fColor);Dispzf((uchar)ccode,curv,curh+8,bColor,fColor);}
}



void Dispsz16(u16 dcode,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor) 
{
	u32 xpos =0, ypos = 0,vpos =0, hpos = 0,pos;
  u16 temp;
  u8  i,n,j,fsz[16],shuw[4];
	
	vpos=xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
  hpos=ypos = ypos + curh;
	
  shuw[0]=dcode/1000; //¸ßÎ»
  shuw[1]=(dcode-shuw[0]*1000)/100;
  shuw[2]=(dcode-shuw[0]*1000-shuw[1]*100)/10;
  shuw[3]=dcode-shuw[0]*1000-shuw[1]*100-shuw[2]*10;
  for(n=4-num;n<4;n++)
      {
			 pos=0x1D560+ (shuw[n]+0x30)*16;
	     SPI_Flash_Read(fsz,pos,16);
			 xpos=vpos;ypos=hpos;		
       for(j = 0; j < 16; j++)
         {temp=fsz[j]; 			 
	        for(i=0;i<8;i++)
					   {temp <<= 1;
	            if((temp&0x0100)!=0) 
					       { /* Write data value to SDRAM memory */
                  *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
                 }
	            else                 
                 { /* Write data value to SDRAM memory */
                  *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
                 }
              ypos++; //ÁÐÏñËØ¼Ó1       
	           }
					ypos += 792;//ÐÐÏñËØ¼Ó1
         }
       hpos=hpos+8;
      }
}



void Dispzf(u8 ncode,u16 curv, u16 curh, u16 bColor, u16 fColor)
{
 u32 xpos =0, ypos = 0,pos;
 u16 temp;  //Paddr,Baddr
 u8  i,j,fsz[16];

 xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
 ypos = ypos + curh;
	
 pos=0x1D560+ ncode*16;
 SPI_Flash_Read(fsz,pos,16);	
 for(i=0; i<16; i++)       
    {temp=fsz[i];
     for(j=0;j<8; j++)    
        {temp <<= 1;
	       if((temp&0x0100)!=0) 
					  { /* Write data value to SDRAM memory */
              *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
            }
	       else                 
            { /* Write data value to SDRAM memory */
              *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
            }
        ypos++;
       }
    ypos += 792;
   }
}


void Disphzs24(u16 ccode,u16 curv,u16 curh,u16 bColor,u16 fColor)
{ 
  u32 xpos =0, ypos = 0,temp,pos,vpos =0;
  u8  i,j,fsz[72];
  if((ccode<=0xd7f9) && (ccode>=0xb0a1))
  {	vpos = xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
		ypos = ypos + curh;
		pos=0x1E560+(((ccode>>8)-0xb0)*94+(ccode&0x00ff)-0xa1)*72;
		SPI_Flash_Read(fsz,pos,72);
	}
	else
	{	  for(i=0;i<72;i++)	fsz[i]=0;
	}
  for(i=0;i<24;i++)	    
         {temp=(fsz[3*i]<<16)+(fsz[3*i+1]<<8)+fsz[3*i+2]; //×óÉÏ8ÐÐ+×óÖÐ8ÐÐ+×óÏÂ8ÐÐ 
					xpos=vpos;	
          for(j=0;j<24;j++)
					   {temp <<= 1;
						  if((temp&0x1000000)!=0) 
					      { /* Write data value to SDRAM memory */
                 *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
                }
							else                 
                { /* Write data value to SDRAM memory */
                 *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
                }
              xpos +=LCD_PIXEL_WIDTH*2;//ÐÐÏñËØ¼Ó1600
						 }
          ypos ++; //ÁÐÏñËØ¼Ó1  
         }			 
}



void Dispzf24(u8 ncode,u16 curv, u16 curh, u16 bColor, u16 fColor)
{
 u32 xpos =0, ypos = 0,pos,temp;
 u8  i,j,fsz[48];

 xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
 ypos = ypos + curh;	
 pos=0x606E0+ncode*48;
 SPI_Flash_Read(fsz,pos,48);
 for(i=0; i<24; i++)       
    {temp=(fsz[2*i]<<8)+fsz[2*i+1]; //×ó8ÁÐ+ÓÒ8ÁÐ	 
     for(j=0;j<16;j++)    
        {temp <<= 1;
	       if((temp&0x10000)!=0) 
					  { /* Write data value to SDRAM memory */
              *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
            }
	       else                 
            { /* Write data value to SDRAM memory */
              *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
            }
         ypos ++;
       }
     ypos +=784;
    }	  
} 



void Dispsz24(u16 dcode,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor) 
{
	u32 xpos =0, ypos = 0,vpos =0, hpos = 0,pos,temp;
  u8  i,n,j,fsz[48],shuw[4];
	
	vpos=xpos = curv*LCD_PIXEL_WIDTH*2;//Ã¿¸öÏñËØÕ¼ÓÃ2¸ö×Ö½Ú£¨16Î»ÑÕÉ«£©
  hpos=ypos = ypos + curh;
	
  shuw[0]=dcode/1000; //¸ßÎ»
  shuw[1]=(dcode-shuw[0]*1000)/100;
  shuw[2]=(dcode-shuw[0]*1000-shuw[1]*100)/10;
  shuw[3]=dcode-shuw[0]*1000-shuw[1]*100-shuw[2]*10;
  for(n=4-num;n<4;n++)
      {pos=0x606E0 + (shuw[n]+0x30)*48;
	     SPI_Flash_Read(fsz,pos,48);
			 xpos=vpos;ypos=hpos;		
       for(i = 0; i < 24; i++)
         {temp=(fsz[2*i]<<8)+fsz[2*i+1]; //×ó8ÁÐ+ÓÒ8ÁÐ	 			 
	        for(j=0;j<16;j++)
					   {temp <<= 1;
	            if((temp&0x10000)!=0) 
					       { /* Write data value to SDRAM memory */
                  *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = fColor;
                 }
	            else                 
                 { /* Write data value to SDRAM memory */
                  *(__IO uint16_t*) (CurrentFrameBuffer + (2*ypos) + xpos) = bColor;
                 }
              ypos++; //ÁÐÏñËØ¼Ó1       
	           }
					ypos += 784;//ÐÐÏñËØ¼Ó1
         }
       hpos=hpos+16;
      }
}



void Dispstr(u8 begin,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor)
{u8 i,x; u16 xx;
 for(i=0;i<num;i++)
   {x=*(dstr+begin); 
    if(x>0x80){xx=x<<8;        
			         x=*(dstr+begin+1);
               xx=xx+x;
			         Disphzs24(xx,curv,curh,bColor,fColor);
               dstr++;
              }
    else Dispzf24(x,curv,curh,bColor,fColor);
    dstr++;
	  curh=curh+24;
   }
 return;
}
void Dispstr16(u8 begin,u8 num,u16 curv,u16 curh,u16 bColor,u16 fColor)
{ u8 i,x; u16 xx;
	for(i=0;i<num;i++)
  { x=*(dstr+begin); 
    if(x>0x80)
		{	xx=x<<8;        
			x=*(dstr+begin+1);
      xx=xx+x;
			Disphzs(xx,curv,curh,bColor,fColor);
      dstr++;
	    curh=curh+16;
    }
    else
		{	Dispzf(x,curv,curh,bColor,fColor);
			curh=curh+8;
		}	
    dstr++;

  }
  return;
}

void Ds1302Write(uchar addr, uchar dat)
{
	u8 n;
	FI2C_RST = 0;
	 Delay16s(20,2);	//1.4us
	DS1302_DIR(1);
	FI2C_SCLK = 0;//??SCLK?????
	Delay16s(20,2);
	FI2C_RST = 1; //???RST(CE)?????
	Delay16s(2,2);	//_nop_();

	for (n=0; n<8; n++)//??????????
	{
		FI2C_DSIO = addr & 0x01;//?????????
		addr >>= 1;
		FI2C_SCLK = 1;//???????,DS1302????
		Delay16s(20,2);
		FI2C_SCLK = 0;
		Delay16s(20,2);
	}
	for (n=0; n<8; n++)//??8???
	{
		FI2C_DSIO = dat & 0x01;
		dat >>= 1;
		FI2C_SCLK = 1;//???????,DS1302????
		Delay16s(20,2);
		FI2C_SCLK = 0;
		Delay16s(20,2);
	}	
		 
	FI2C_RST = 0;//??????
	Delay16s(20,2);
}

u8 Ds1302Read(u8 addr)
{
	u8 n,dat,dat1;
	FI2C_RST = 0;
	Delay16s(20,2);
	DS1302_DIR(1);

	FI2C_SCLK = 0;//??SCLK?????
	Delay16s(20,2);
	FI2C_RST = 1;//???RST(CE)?????
	Delay16s(20,2);

	for(n=0; n<8; n++)//??????????
	{
		FI2C_DSIO = addr & 0x01;//?????????
		addr >>= 1;
		Delay16s(20,2);
		FI2C_SCLK = 1;//???????,DS1302????
		Delay16s(20,2);
		FI2C_SCLK = 0;//DS1302????,????
		Delay16s(20,2);
	}
	Delay16s(20,2);
	DS1302_DIR(0);  //io=>input

	for(n=0; n<8; n++)//??8???
	{
		dat1 = FI2C_DSII;//????????
		dat = (dat>>1) | (dat1<<7);
		Delay16s(20,2);
		FI2C_SCLK = 1;
		Delay16s(20,2);
		FI2C_SCLK = 0;//DS1302????,????
		Delay16s(20,2);
	}
	DS1302_DIR(1);
	FI2C_RST = 0;
	Delay16s(20,2);	//???DS1302????????
	FI2C_SCLK = 1;
	Delay16s(20,2);
	FI2C_DSIO = 0;
	Delay16s(20,2);
	FI2C_DSIO = 1;
	Delay16s(20,2);
	return dat;	
}
void Ds1302Init()
{
//	u8 n;
	Ds1302Write(0x80,0x00);		 //clock run
	Ds1302Write(0x8E,0x00);		 //Write_Enable
//	for (n=0; n<6; n++)        //??6????????:??????
//	{		Ds1302Write(WRITE_RTC_ADDR[n],1);	
//     		Ds1302Write(WRITE_RTC_ADDR[n],FI2C_time[n]);	
//	}
//	Ds1302Write(0x8E,0x80);		 //Write_Disable
}
void Ds1302ReadTime(void)
{	u8 i,j,r0;

	r0=Ds1302Read(0x8d);
	i=r0>>4;i=i*10;j=(r0&0x0f)+i;FI2C_time[0]=j; //year
	r0=Ds1302Read(0x89);
	i=r0>>4;i=i*10;j=(r0&0x0f)+i;FI2C_time[1]=j;//mon
	r0=Ds1302Read(0x87);
	i=r0>>4;i=i*10;j=(r0&0x0f)+i;FI2C_time[2]=j;  //day
	r0=Ds1302Read(0x85);
	i=r0>>4;i=i*10;j=(r0&0x0f)+i;FI2C_time[3]=j; //hour
	r0=Ds1302Read(0x83);
	i=r0>>4;i=i*10;j=(r0&0x0f)+i;FI2C_time[4]=j; //min	
//	r0=Ds1302Read(0x81);
//	i=r0>>4;i=i*10;j=(r0&0x0f)+i;time[5]=j; //sec			
}
uchar Ds1302SetTime(uchar *p)
{	
	u8 n,x,y,r0,stime[6];
	
	Ds1302Write(0x8E,0X00);		 //?????,?????????
	for (n=0; n<6; n++)        //??6????????:?????
	  {
     stime[n]=*(p+n);
	   x=stime[n]/10;y=stime[n]-(x*10);
     r0=x*16+y;
		 Ds1302Write(WRITE_RTC_ADDR[n],r0);	
	  }
	Ds1302Write(0x8E,0x80);		 //???????
	Delay16s(200,200);  //1400 us
	Ds1302ReadTime();	//????	
	for(n=0;n<5;n++)
	  {
		 if(FI2C_time[n]!=stime[n]) return 0;
	  }
	return 0xff;
	
}




void USB_HOST_MAIN(u8 num)
{	u8 i=0,j,m=0,k=0,shuz[3];
	u16 cb=0;
	//´Ë´¦Ôö¼ÓÏÔÊ¾UÅÌ¶ÁÈ¡½çÃæ
	bsp_delay=0;usbresfg=0;
	USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);//USB³õÊ¼»¯	
  do
	{	 // Host Task handler 
		if(usbresfg==0) 
		{	dstr=XCfile; Dispstr16(1,4,TiShi_y+17,TiShi_x,Blue,White);	//XS
//			printf("\r\n UÅÌÕýÔÚÁ¬½Ó£¡\r\n"); 
			USBH_Process(&USB_OTG_Core, &USB_Host);
		}
		if(usbresfg==1)
		{	usbresfg=2;
			dstr=XCfile; Dispstr16(10,4,TiShi_y+17,TiShi_x,Blue,White);	//XS
//			printf("\r\nÇý¶¯Õý³££¡\r\n");
			bsp_delay=500; 
		}       		
		if(tick1s>=20) //1s_1 time
		{	tick1s=0;
			if(m==0){m=1;LedXT_on();}		//ÏµÍ³¹ÊÕÏµÆÉÁË¸
			else    {m=0;LedXT_off();}
		}			
	}while(bsp_delay<600);		//30SÑÓÊ±
	 // Mount a logical drive 
	if (f_mount(2,&fs) == FR_OK )	
	{	dstr=XCfile; Dispstr16(19,4,TiShi_y+17,TiShi_x,Blue,White);	//XS
//		printf("ÎÄ¼þÏµÍ³¹ÒÔØ³É¹¦£¡\r\n");
		goto loop4;
	}
	else
	{	dstr=XCfile; Dispstr16(28,4,TiShi_y+17,TiShi_x,Blue,White);	//XS
//		printf("\r\nÎÄ¼þÏµÍ³¼ÓÔØÊ§°Ü£¡\r\n");
		return;
	}

loop4://´Ë´¦Ôö¼ÓÏÔÊ¾Ñ¡ÔñÎÄ¼þ½çÃæ
//num=40;//´Ë´¦Ôö¼Ó°´¼ü¹¦ÄÜÑ¡Ôñ,µ±Ç°¸³Öµnum=40È«²¿¶ÁÈë
//loop3:
//			if(key==xxx)goto loop;//¿ªÊ¼¶ÁÈ¡ÎÄ¼þ
//      if(key==xxx)//ÍË³ö
//				 {f_close(&fil);
//					f_mount(2,NULL);
//          printf("\r\nµ¯³öUÅÌ£¡\r\n");//µ¯³öUÅÌ	
//          Delay1ms(2000);						 
//				  return;
//				 }
//			goto loop3;
	Read_USB_file(num);						
	f_close(&fil);
	f_mount(2,NULL);
	dstr=XCfile; Dispstr16(37,4,TiShi_y+17,TiShi_x,Blue,White);	//XS
//	printf("\r\nµ¯³öUÅÌ£¡\r\n");//µ¯³öUÅÌ	
	Delay1ms(2000);						 
}

void Read_USB_file(u8 num){
	u8 i=0,x,y,j;
	u16 F_long;	
	u32 k,waddr;
	USART1_SendByte(num);
	Delay16s(20,20);
	for(k=0;k<51207; k++)downb[k]=0;
	dstr=XCfile;
	Dispstr16(46,4,TiShi_y+17,TiShi_x,Blue,White);
//	printf("----- ¼´½«½øÐÐÎÄ¼þ¶ÁÈ¡-----\r\n");	
	//¶ÁÏÔÊ¾±à³Ì011/021/031
	if(num==11||num==40)
	{	if(f_open(&fil, "2:sd2311/sd2300.011", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=51207)goto loop11;
				for(k=6;k<0xc806;k=k+32)  //yuan 8198
				{	for(j=6;j<26;j++)
					{	i=downb[k+j];if(i<0xa0&&i>0x9a)downb[k+j]=downb[k+j]+0x60;}
				}
				waddr=0xa0000+0xc800;
				SPI_Flash_Read(downb+0xd000,waddr,0x800); //9loop_half read
				waddr=0xa0000;
				for(j=0;j<=13;j++)	{ SPI_Flash_Erase_Sector(160+j);}
				SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
				waddr=0xa0000+0xc800;
				SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
           //for(j=0;j<fnum;j++)USART1_SendByte(downb[j]);//							
				dstr=XCMenu;
				Dispstr16(1,2,TiShi_y+17,TiShi_x,Blue,White);	//XS
				dstr="011_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.011ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);
			}
		}	
		else
		{	dstr="011_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop11:
		f_close(&fil);	
		Delay1ms(2000);
	}	
	if(num==21||num==40)
	{	if(f_open(&fil, "2:sd2311/sd2300.021", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=51207)	goto loop21; 
				for(k=6;k<0xc806;k=k+32)  //yuan 8198
				{	for(j=6;j<26;j++)
					{	i=downb[k+j];if(i<0xa0&&i>0x9a)downb[k+j]=downb[k+j]+0x60;}
				}
				waddr=0xa0000+0xc000;
				SPI_Flash_Read(downb+0xd000,waddr,0x800); //8loop_half read
				waddr=0xa0000+0xc800;
				for(j=13;j<=26;j++)	{ SPI_Flash_Erase_Sector(160+j);}
				SPI_Flash_Write_NoCheck(downb+6,waddr,0xc800);
				waddr=0xa0000+0xc000;
				SPI_Flash_Write_NoCheck(downb+0xd000,waddr,0x800);
				dstr=XCMenu;
				Dispstr16(1,2,TiShi_y+17,TiShi_x,Blue,White);	//XS
				dstr="021_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.021ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);
			}						
		}
		else
		{	dstr="021_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop21:
		f_close(&fil);	
		Delay1ms(2000);
	}
	if(num==31||num==40)
	{	if(f_open(&fil, "2:sd2311/sd2300.031", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=12807)goto loop31;
				for(k=6;k<0x3206;k=k+32)  //yuan 8198
				{	for(j=6;j<26;j++)
					{	i=downb[k+j];if(i<0xa0&&i>0x9a)downb[k+j]=downb[k+j]+0x60;}
				}
				waddr=0xa0000+0x19000;
				for(j=27;j<=34;j++)	{ SPI_Flash_Erase_Sector(160+j);}
				SPI_Flash_Write_NoCheck(downb+6,waddr,0x6400);
				dstr=XCMenu;
				Dispstr16(1,2,TiShi_y+17,TiShi_x,Blue,White);	//XS
				dstr="031_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.031ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);
			}						
		}
		else
		{	dstr="031_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop31:
		f_close(&fil);	
		Delay1ms(2000);				 
	}	
	//¶ÁÇøÓò±à³Ì004	
	if(num==4||num==40)			//ÇøÓò±à³Ì004_9e000_9e600 6x256=1536=600h 1B
	{	if(f_open(&fil, "2:sd2311/sd2300.004", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");   
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=1543)goto loop4;
				SPI_Flash_Erase_Sector(158);
				SPI_Flash_Write_NoCheck(downb+6,0x9e000,0x600);
				dstr=XCMenu;
				Dispstr16(55,2,TiShi_y+17,TiShi_x,Blue,White);	//XS
				dstr="004_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.004ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);					 
			}	
		}
		else
		{	dstr="004_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop4:
		f_close(&fil);
		Delay1ms(2000);						 
	}
	//¶ÁÉè±¸Ãû³Æ005
	if(num==5||num==40)			//Éè±¸Ãû³Æ005_9f000_9ffff
	{	if(f_open(&fil, "2:sd2311/sd2300.005", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);				 
				if(fnum!=2055)goto loop5;	
				for(k=6;k<0x806;k++)
				{	i=downb[k];if(i<0xa0&&i>0x9a)downb[k]=downb[k]+0x60;}
				SPI_Flash_Erase_Sector(159);
				SPI_Flash_Write_NoCheck(downb+6,0x9f000,0x800);
				dstr=XCMenu;
				Dispstr16(19,2,TiShi_y+17,TiShi_x,Blue,White);	//XS
				dstr="005_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.005ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);								 
			}	
		}
		else
		{	dstr="005_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop5:
		f_close(&fil);
		Delay1ms(2000);						 
	}		
	// ¶Á×ÜÏß±à³Ì003
	if(num==3||num==40)		//×ÜÏß±à³Ì_9c000_9d000 2x256x8=4096=1000h
	{	if(f_open(&fil, "2:sd2311/sd2300.003", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=4103)goto loop3;	
				SPI_Flash_Erase_Sector(156);
				SPI_Flash_Write_NoCheck(downb+6,0x9c000,0x1000);
				dstr=XCMenu;
				Dispstr16(37,2,TiShi_y+17,TiShi_x,Blue,White);
				dstr="003_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.003ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);							 
			}	
		}
		else
		{	dstr="003_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop3:
		f_close(&fil);
		Delay1ms(2000);					 
	}	
	//¶ÁÉù¹â¹ã²¥½»Ìæ007
	if(num==7||num==40)		//Éù¹â¹ã²¥007_9d000_9e000 2x256x8=4096=1000h
	{	if(f_open(&fil, "2:sd2311/sd2300.007", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	//printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
				if(fnum!=2056)goto loop7;	
				SPI_Flash_Erase_Sector(157);
				SPI_Flash_Write_NoCheck(downb+6,0x9d000,0x1000);
				dstr=XCMenu;
				Dispstr16(46,2,TiShi_y+17,TiShi_x,Blue,White);
				dstr="007_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·.007ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);							 
			}	
		}
		else
		{	dstr="007_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
loop7:
		f_close(&fil);
		Delay1ms(2000);					 
	}				
	//¶ÁÁª¶¯±à³Ì002-
	if(num==2||num==40)		//¶ÁÈ¡Áª¶¯±à³Ì002 _e0000_e8000,ef000_ef800		
	{	if(f_open(&fil, "2:sd2311/sd2300.002", FA_OPEN_EXISTING | FA_READ) == FR_OK)
		{	//printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
			{	F_long =downb[4]*256 +downb[5]+6;
				SPI_Flash_Erase_Block(14);
				SPI_Flash_Write_NoCheck(downb+6,0xef000,0x800);
USART1_SendByte(downb[4]);Delay16s(20,20);USART1_SendByte(downb[5]);Delay16s(20,20);
				SPI_Flash_Write_NoCheck(downb+6+0x800,0xe0000,(F_long));
				dstr=XCMenu;
				Dispstr16(10,2,TiShi_y+17,TiShi_x,Blue,White);
				dstr="002_OK";
				Dispstr16(1,6,TiShi_y+17,TiShi_x+32,Blue,White);	//XS
//				printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum); 
//				printf("¡·.002ÏÂÔØ³É¹¦¡£\r\n");
				Delay1ms(2000);					 
			}	
		}
		else
		{	dstr="002_OP_ERR"; Dispstr16(1,10,TiShi_y+17,TiShi_x,Blue,White);	//XS
			//printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
		}
	  f_close(&fil);
    Delay1ms(2000);						 
	}
	//¶ÁÇøÓòÁª¶¯±à³Ì006
//	if(num==6||num==40)//¶ÁÈ¡ÇøÓò¼¯ÖÐÁª¶¯±à³Ì		
//	{	if(f_open(&fil, "2:sd2311/sd2300.006", FA_OPEN_EXISTING | FA_READ) == FR_OK)
//		{	printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
//			if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
//			{	printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);  
//				printf("¡·.006ÏÂÔØ³É¹¦¡£\r\n");
//				Delay1ms(2000);					 
//			}	
//		}
//		else
//		{	printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
//		}
//	  f_close(&fil);
//    Delay1ms(2000);						 
//	}
// ¶ÁÏµÍ³ÉèÖÃ000
//	if(num==0||num==40)
//  {	if(f_open(&fil, "2:sd2311/sd2300.000", FA_OPEN_EXISTING | FA_READ) == FR_OK)
//	     {printf("¡·´ò¿ªÎÄ¼þ³É¹¦¡£\r\n");
//		    if(f_read(&fil, downb, sizeof(downb), &fnum) == FR_OK)
//          {printf("¡·¶ÁÈ¡³É¹¦,¶Áµ½×Ö½ÚÊý¾Ý£º%d\r\n",fnum);
//           if(fnum!=138)goto loop0;	
//				 //Êý¾ÝÒÑ¶ÁÈëdownb[],Ìí¼ÓFLASHÐ´Èë´¦Àí³ÌÐò			
//         //for(j=0;j<fnum;j++)USART1_SendByte(downb[j]);//////////////////////////////////////////	
//	         printf("¡·.000ÏÂÔØ³É¹¦¡£\r\n");
//				   Delay1ms(2000);							 
//          }	
//	     }
//	   else{
//		      printf("£¡£¡´ò¿ªÎÄ¼þÊ§°Ü¡£\r\n");
//	       }
//		loop0:
//			f_close(&fil);
//      Delay1ms(2000);					 
//	}			
}

