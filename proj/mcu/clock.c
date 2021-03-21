
#include "../tl_common.h"

#if MODULE_AUDIO_ENABLE
#define CLK_SBC_ENABLE		1
#define CLK_AUD_ENABLE		1
#define CLK_DFIFO_ENABLE	1
#define CLK_USB_ENABLE		1
#endif

#if (MODULE_USB_ENABLE)
#define CLK_AUD_ENABLE		1
#endif

#ifndef CLK_SBC_ENABLE
#define CLK_SBC_ENABLE		1
#endif
#ifndef CLK_AUD_ENABLE
#define CLK_AUD_ENABLE		1
#endif
#ifndef CLK_DFIFO_ENABLE
#define CLK_DFIFO_ENABLE	1
#endif
#ifndef CLK_USB_ENABLE
#define CLK_USB_ENABLE		(APPLICATION_DONGLE)
#endif

enum{
	CLK_EN_TYPE = (CLK_SBC_ENABLE ? FLD_CLK_SBC_EN:0) | (CLK_AUD_ENABLE ? FLD_CLK_AUD_EN : 0)	| (CLK_DFIFO_ENABLE ? FLD_CLK_DIFIO_EN : 0),
};

void clock_init(){

#if(MCU_CORE_TYPE == MCU_CORE_8368)
	//when mcu back from deep, it must reset ldo to make sure pll clock to work, and rf will stable
	analog_write(0x02, 1);					//Digital LDO output voltage trim
	u8 ana_05 = analog_read(0x05);
	int cnt = 10;
	while(cnt--){
		CLOCK_DLY_10_CYC;					//wait for 10us
	}
	analog_write(0x02, 0x05);				//reset LDO
	analog_write(0x05, ana_05 | BIT(7));	//power on PLL clock
	analog_write(0x05, ana_05 & (~BIT(7)));	//power down PLL clock
	cnt = 20;
	while(cnt--){
		CLOCK_DLY_10_CYC;
	}
#endif

	reg_rst_clk0 = 0xff000000 | (CLK_USB_ENABLE ? FLD_CLK_USB_EN: 0);

#if(CLOCK_SYS_TYPE == CLOCK_TYPE_PLL)
	reg_clk_sel = MASK_VAL(FLD_CLK_SEL_DIV, (CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S), FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV);
#elif(CLOCK_SYS_TYPE == CLOCK_TYPE_PAD)

	//STATIC_ASSERT(CLK_FHS_MZ == 32);
	#if(CLOCK_SYS_CLOCK_HZ == 12000000)
		reg_clk_sel = 0x40;
	#else
		#error clock not set properly
	#endif
	
#elif(CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
	#if(MCU_CORE_TYPE == MCU_CORE_8267)
		#if(CLOCK_SYS_CLOCK_HZ == 32000000)
			reg_fhs_sel = 0;
			reg_clk_sel = 0x80;	// bit[7] must be "1"
		#elif(CLOCK_SYS_CLOCK_HZ == 16000000)
			reg_fhs_sel = 0;
			reg_clk_sel = 0xa2;
		#elif(CLOCK_SYS_CLOCK_HZ == 8000000)
			reg_fhs_sel = 0;
			reg_clk_sel = 0xa4;
		#else
			#error clock not set properly
		#endif
	#else
		#if(CLOCK_SYS_CLOCK_HZ == 32000000)
			reg_fhs_sel = 0;
			reg_clk_sel = 0;	// must be zero
		#elif(CLOCK_SYS_CLOCK_HZ == 16000000)
			reg_fhs_sel = FHS_SEL_32M_OSC;
			reg_clk_sel = MASK_VAL(FLD_CLK_SEL_DIV, 2, FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV);
		#elif(CLOCK_SYS_CLOCK_HZ == 8000000)
			reg_fhs_sel = FHS_SEL_32M_OSC;
			reg_clk_sel = MASK_VAL(FLD_CLK_SEL_DIV, 4, FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV);
		#else
			#error clock not set properly
		#endif
	#endif
#else
	#error clock not set properly
#endif
	//reg_clk_en = 0xff | CLK_EN_TYPE;
	reg_tmr_ctrl = MASK_VAL(FLD_TMR0_EN, 1
		, FLD_TMR_WD_CAPT, (MODULE_WATCHDOG_ENABLE ? (WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF):0)
		, FLD_TMR_WD_EN, (MODULE_WATCHDOG_ENABLE?1:0));
}


_attribute_ram_code_ void sleep_us (u32 us)
{
	u32 t = clock_time();
	while(!clock_time_exceed(t, us)){
	}
}

#if (__PROJECT_DONGLE__ || __PROJECT_MOUSE__)
void bbpll_reset(void){

#if 0		//12M crystal
	analog_write(0x82, 0x0);
#else		//16M crystal
	analog_write(0x82, 0x14);
#endif

	//when mcu back from deep, it must reset ldo to make sure pll clock to work, and rf will stable
	analog_write(0x02, 1);					//Digital LDO output voltage trim
	u8 ana_05 = analog_read(0x05);
	int cnt = 30;
	while(cnt--){
		CLOCK_DLY_10_CYC;					//wait for 10us
	}
	analog_write(0x02, 0x05);				//reset LDO
	analog_write(0x05, ana_05 | BIT(7));	//power on PLL clock
	analog_write(0x05, ana_05 & (~BIT(7)));	//power down PLL clock
	cnt = 60;
	while(cnt--){
		CLOCK_DLY_10_CYC;					//wait for 30us
	}

}
#endif
