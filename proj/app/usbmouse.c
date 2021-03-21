
#include "../tl_common.h"
#if(USB_MOUSE_ENABLE)

#include "usbmouse.h"
#include "../drivers/usb.h"
#include "../drivers/usbhw.h"
#include "../drivers/usbhw_i.h"
#include "../../vendor/common/rf_frame.h"

#ifndef	USB_MOUSE_REPORT_SMOOTH
#define	USB_MOUSE_REPORT_SMOOTH	   1
#endif


#define  USBMOUSE_BUFF_DATA_NUM    16
static mouse_data_t mouse_dat_buff[USBMOUSE_BUFF_DATA_NUM];

static u8  usbmouse_wptr, usbmouse_rptr;
static u32 usbmouse_not_released;
static u32 usbmouse_data_report_time;


int sm_sum_x, sm_pre_x, sm_sum_y, sm_pre_y;
void iir_smoother(signed char *data_x, signed char *data_y){
	sm_sum_x = sm_sum_x - sm_pre_x + *data_x;
	sm_pre_x = sm_sum_x/2;
	*data_x = sm_pre_x;

    sm_sum_y = sm_sum_y - sm_pre_y + *data_y;
	sm_pre_y = sm_sum_y/2;
	*data_y = sm_pre_y;
}

static inline void iir_smoother_clear( void ){
	sm_sum_x = 0;
	sm_pre_x = 0;

    sm_sum_y = 0;
	sm_pre_y = 0;
}

const u8 sm_dyn_pth1 = 6;
const u8 sm_dyn_pth2 = 4;
u8 adaptive_smoother(signed char *data_x, signed char *data_y ){
    static u8 asm_flg = 0;    
    static u32 sm_last_smoother_tick = 0;
    
    //auto clear asm sum when no data for a long time
    if ( asm_flg && clock_time_exceed(sm_last_smoother_tick, 100000) ){           
        asm_flg = 0;
        iir_smoother_clear ( );
    }
    if ( !asm_flg ){
        if ( (abs(*data_x) > sm_dyn_pth1) || (abs(*data_y) > sm_dyn_pth1) ){
            asm_flg = 1;
            iir_smoother ( data_x, data_y );
        }
        else{
            asm_flg = 0;
            iir_smoother_clear ( );
        }
    }
    else{
        if ( (abs(*data_x) < sm_dyn_pth2) && (abs(*data_y) < sm_dyn_pth2) ){
           asm_flg = 0;
           iir_smoother_clear ( );
        }
        else{
           asm_flg = 1;
           iir_smoother ( data_x, data_y );
        }
    }
    
    if ( asm_flg ){
        sm_last_smoother_tick = clock_time();
    }
    
    return asm_flg;
}


#if 0
u8 mouse_button_debounce( u8 input, u8 debouce_len){
    static u8 btn_bak_bak = 0;    
    static u8 btn_bak = 0;
    u8 btn = btn_bak_bak & btn_bak & input;
    btn_bak_bak = btn_bak;
    btn_bak = input;
    return btn;
}
#else
static inline u8 mouse_button_debounce(u8 btn_cur, u8 debouce_len){
    static u8 btn_cnt = 0;
    static u8 btn_bak = 0;
	if(	(btn_cnt >= debouce_len) && (btn_bak != btn_cur) ){
		btn_bak = btn_cur;
		btn_cnt = 0;
	}else{
		if(btn_bak != btn_cur)
			btn_cnt ++;
		else
			btn_cnt = 0;
	}
    return btn_bak;
}
#endif


#if (SIMULATE_CAVY_DONGLE_ENABLE)
void usbmouse_add_frame (rf_packet_cavy_mouse_t *packet_mouse)
#elif (__PROJECT_DONGLE_ENC_8366__)
void usbmouse_add_frame (rf_packet_mouse_enc_t *packet_mouse)
#else
void usbmouse_add_frame (rf_packet_mouse_t *packet_mouse,u8 interpolate_en)
#endif
{

	u8 *mdata_ptr;

	u8 new_data_num = packet_mouse->pno;  //根据pno 获得最新数据的个数
	for(u8 i=0;i<new_data_num;i++)
	{

		//sensor data smooth
		mdata_ptr = (u8 *)(&packet_mouse->data[i*sizeof(mouse_data_t)]);
		*(mdata_ptr+0) = mouse_button_debounce( *(mdata_ptr+0), MOUSE_BUTTON_DEBOUNCE - 1 );
		if(interpolate_en){
			s8 mdata_x = *(mdata_ptr+1);
			s8 mdata_y = *(mdata_ptr+2);

			*(mdata_ptr+1) = mdata_x - (mdata_x >>1);
			*(mdata_ptr+2) = mdata_y - (mdata_y >>1);

			if ( *(mdata_ptr+1) || *(mdata_ptr+2) ){
				adaptive_smoother ( (mdata_ptr+1), (mdata_ptr+2) );
			}
			memcpy4((int*)(&mouse_dat_buff[usbmouse_wptr++]), (int*)mdata_ptr, sizeof(mouse_data_t));

//			usbmouse_wptr &= USBMOUSE_BUFF_DATA_NUM - 1;

			*(mdata_ptr+1) = (mdata_x >>1);
			*(mdata_ptr+2) = (mdata_y >>1);
			*(mdata_ptr+3) = 0;
		}

		if ( *(mdata_ptr+1) || *(mdata_ptr+2) ){
			adaptive_smoother ( (mdata_ptr+1), (mdata_ptr+2) );
		}
		memcpy4((int*)(&mouse_dat_buff[usbmouse_wptr]), (int*)(mdata_ptr), sizeof(mouse_data_t));

		BOUND_INC_POW2(usbmouse_wptr,USBMOUSE_BUFF_DATA_NUM);
		if(usbmouse_wptr == usbmouse_rptr)
		{
				BOUND_INC_POW2(usbmouse_rptr,USBMOUSE_BUFF_DATA_NUM);
				break;
		}
	}
}


void usbmouse_release_check(){
	if(usbmouse_not_released && clock_time_exceed(usbmouse_data_report_time, USB_MOUSE_RELEASE_TIMEOUT)){
	    u32 release_data = 0;

	    if(usbmouse_hid_report(USB_HID_MOUSE, (u8*)(&release_data), MOUSE_REPORT_DATA_LEN)){
		    usbmouse_not_released = 0;
	    }
	}
}

u8 button_last;

void usbmouse_report_frame(){
	u8  data_check = 0;
	u8 	button_check;
	int ret;
#if 	USB_MOUSE_REPORT_SMOOTH
	static u32 tick = 0;
	if(usbhw_is_ep_busy(USB_EDP_MOUSE)) {
			tick = clock_time ();
	}

	u8 diff = (usbmouse_wptr - usbmouse_rptr) & (USBMOUSE_BUFF_DATA_NUM - 1);
    //buff cycle
    if ( usbmouse_rptr > usbmouse_wptr )
        diff = usbmouse_rptr - usbmouse_wptr + (USBMOUSE_BUFF_DATA_NUM - 1);
	if (diff < 3 && !clock_time_exceed (tick, 5000)) {
		return;
	}
#endif

	if(usbmouse_wptr != usbmouse_rptr){

		if((0 == mouse_dat_buff[usbmouse_rptr].x) && (0 == mouse_dat_buff[usbmouse_rptr].y) && (0 == mouse_dat_buff[usbmouse_rptr].wheel)){
			data_check = 1;
		}
		if(button_last != mouse_dat_buff[usbmouse_rptr].btn){
			button_check = 0;
		}
		else{
			button_check = 1;
		}
		if(!(button_check && data_check)){
			ret = usbmouse_hid_report(USB_HID_MOUSE,(&mouse_dat_buff[usbmouse_rptr].btn), MOUSE_REPORT_DATA_LEN);
		}

		if(ret && (button_check == 0)){
			button_last = mouse_dat_buff[usbmouse_rptr].btn;
		}
		if(ret ){
			BOUND_INC_POW2(usbmouse_rptr,USBMOUSE_BUFF_DATA_NUM);
		}
		usbmouse_data_report_time = clock_time();
	}
	else{
		if(button_last && clock_time_exceed(usbmouse_data_report_time, USB_MOUSE_RELEASE_TIMEOUT)){
			mouse_button_debounce(0, 0);
			usbmouse_not_released = 1;
			button_last = 0;
		}
	}
	return;
}


int usbmouse_hid_report(u8 report_id, u8 *data, int cnt){
	//unsigned char crc_in[8];
	//unsigned short crc;
	//unsigned int crch;

#if(0)
	static int  previous_busy_flag = 0;
	static u32	usb_busy_start_time;
#endif


    assert(cnt<8);

    if (!reg_usb_host_conn)
    {
    	reg_usb_ep_ctrl(USB_EDP_MOUSE) = 0;
    	return 0;
    }

	if(usbhw_is_ep_busy(USB_EDP_MOUSE)){
#if(0)
		if(!previous_busy_flag){   //previous  not  busy
			usb_busy_start_time = clock_time();
		}
		if(clock_time_exceed(usb_busy_start_time,5000000)){  //5s busy  reset
			write_reg8(0x80006f,0x20);  //reset
		}
		previous_busy_flag = 1;
#endif
		return 0;
	}

#if(0)
	previous_busy_flag = 0;  //not busy
#endif

	reg_usb_ep_ptr(USB_EDP_MOUSE) = 0;

	// please refer to usbmouse_i.h mouse_report_desc
	extern u8 usb_mouse_report_proto;

	if (!usb_mouse_report_proto) {
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[0];
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[1];
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[2];
	}
	else {
		reg_usb_ep_dat(USB_EDP_MOUSE) = report_id;
		foreach(i, cnt){
			reg_usb_ep_dat(USB_EDP_MOUSE) = data[i];
		}
	}
	reg_usb_ep_ctrl(USB_EDP_MOUSE) = FLD_EP_DAT_ACK;		// ACK

	return 1;
}


void usbmouse_init(){
}

#endif
