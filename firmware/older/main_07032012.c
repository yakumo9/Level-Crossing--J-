#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg;

#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#define CLOSE_SWITCH_PIN	REGISTER_BIT(PIND,0)
#define CLOSE_SWITCH_PORT	REGISTER_BIT(PORTD,0)
#define CLOSE_SWITCH_DDR	REGISTER_BIT(DDRD,0)
#define CLOSE_DCC_PIN 		REGISTER_BIT(PINC,4)
#define CLOSE_DCC_DDR 		REGISTER_BIT(DDRC,4)
#define SENSOR_L_PIN		REGISTER_BIT(PINC,5)
#define SENSOR_L_DDR		REGISTER_BIT(DDRC,5)
#define SENSOR_R_PIN		REGISTER_BIT(PINC,3)
#define SENSOR_R_DDR		REGISTER_BIT(DDRC,3)
//s88出力
#define S88OUT_L_PORT		REGISTER_BIT(PORTD,1)
#define S88OUT_L_DDR		REGISTER_BIT(DDRD,1)
#define S88OUT_R_PORT		REGISTER_BIT(PORTD,2)
#define S88OUT_R_DDR		REGISTER_BIT(DDRD,2)
//警報灯
#define FLASHLIGHT_L_PORT		REGISTER_BIT(PORTB,0)
#define FLASHLIGHT_L_DDR		REGISTER_BIT(DDRB,0)
#define FLASHLIGHT_R_PORT		REGISTER_BIT(PORTD,7)
#define FLASHLIGHT_R_DDR		REGISTER_BIT(DDRD,7)
//遮断機
#define OPENGATE_PORT		REGISTER_BIT(PORTD,5)
#define OPENGATE_DDR		REGISTER_BIT(DDRD,5)
#define CLOSEGATE_PORT		REGISTER_BIT(PORTD,6)
#define CLOSEGATE_DDR		REGISTER_BIT(DDRD,6)

//  波形の分割数 = AVR動作クロック ÷ 256(キャリヤ周波数) ÷ 256(PWMの分解能)
//  [122 = 8,000,000Hz / 256 /256]
//	波形インデックス加算値 = 発生周波数 × 256(16bitの上位8bitを使用) ÷ 波形の分割数
//const	int stepOfWave1	= 1444;	//	= 688 Hz * 256/122
//const	int stepOfWave2	= 1542;	//	= 735 Hz * 256/122

//const	int stepOfWave1	= 1466;	//	= 699 Hz * 256/122
//const	int stepOfWave2	= 1557;	//	= 742 Hz * 256/122

const	int stepOfWave1	= 1478;	//	= 704.7 Hz * 256/122
const	int stepOfWave2	= 1385;	//	= 660.0 Hz * 256/122

//振幅0xffの360度分のサインテーブル(分解能256)
const uint8_t sin_table[] PROGMEM = {
	0x7f,0x82,0x85,0x88,0x8b,0x8f,0x92,0x95,
	0x98,0x9b,0x9e,0xa1,0xa4,0xa7,0xaa,0xad,
	0xb0,0xb2,0xb5,0xb8,0xbb,0xbe,0xc0,0xc3,
	0xc6,0xc8,0xcb,0xcd,0xd0,0xd2,0xd4,0xd7,
	0xd9,0xdb,0xdd,0xdf,0xe1,0xe3,0xe5,0xe7,
	0xe9,0xea,0xec,0xee,0xef,0xf0,0xf2,0xf3,
	0xf4,0xf5,0xf7,0xf8,0xf9,0xf9,0xfa,0xfb,
	0xfc,0xfc,0xfd,0xfd,0xfd,0xfe,0xfe,0xfe,
	0xfe,0xfe,0xfe,0xfe,0xfd,0xfd,0xfd,0xfc,
	0xfc,0xfb,0xfa,0xf9,0xf9,0xf8,0xf7,0xf5,
	0xf4,0xf3,0xf2,0xf0,0xef,0xee,0xec,0xea,
	0xe9,0xe7,0xe5,0xe3,0xe1,0xdf,0xdd,0xdb,
	0xd9,0xd7,0xd4,0xd2,0xd0,0xcd,0xcb,0xc8,
	0xc6,0xc3,0xc0,0xbe,0xbb,0xb8,0xb5,0xb2,
	0xb0,0xad,0xaa,0xa7,0xa4,0xa1,0x9e,0x9b,
	0x98,0x95,0x92,0x8f,0x8b,0x88,0x85,0x82,
	0x7f,0x7c,0x79,0x76,0x73,0x6f,0x6c,0x69,
	0x66,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
	0x4e,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,
	0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
	0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,
	0x15,0x14,0x12,0x10,0x0f,0x0e,0x0c,0x0b,
	0x0a,0x09,0x07,0x06,0x05,0x05,0x04,0x03,
	0x02,0x02,0x01,0x01,0x01,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x02,
	0x02,0x03,0x04,0x05,0x05,0x06,0x07,0x09,
	0x0a,0x0b,0x0c,0x0e,0x0f,0x10,0x12,0x14,
	0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
	0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,
	0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
	0x4e,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,
	0x66,0x69,0x6c,0x6f,0x73,0x76,0x79,0x7c
};

//スピーカーボリューム，アタック音／余韻に使う
volatile	unsigned char	Volume;
volatile	unsigned char	DirVolume;

//関数宣言
int suspend( void );
void check_sensor( void );


//タイマー2オーバーフロー割込み
ISR(TIMER2_OVF_vect)
{
	//波形のインデクス値
	static unsigned int 	IndexOfWave1;
	static unsigned int		IndexOfWave2;
	
	//
	static unsigned int		AlarmVolume;
	unsigned char			AlarmVolumeChar;
	
	//波形データ
	unsigned int wav1;
	unsigned int wav2;
	
	//波形を作る
	wav1	= pgm_read_byte( &sin_table[(IndexOfWave1 >> 8)] );	//
	wav2	= pgm_read_byte( &sin_table[(IndexOfWave2 >> 8)] );	//
	wav1	= wav1 >> 1	;//
	
	//アラーム音の立ち上がり/余韻を作る
	if(DirVolume == 2)
	{
		AlarmVolume	+= 128;
		if(AlarmVolume > 32767) 
		{
			AlarmVolume	= 32767;
			DirVolume	= 1;
		}
	}
	else if(DirVolume == 1)
	{
		if(AlarmVolume > 6144)	AlarmVolume	-= 4;
		else					AlarmVolume	= 6144;
	}
	
	AlarmVolumeChar = AlarmVolume >> 7;
	//2つの波形を加算して(桁が一つ多くなるのでシフトする)カウンタへ入れる。
	OCR2B	= (((wav1 >> 1) + ( wav2 >> 1)) * AlarmVolumeChar) >> 8;
	
	if(DirVolume	== 0)
	{
		if(IndexOfWave1 !=0)
		{
			IndexOfWave1 += stepOfWave1;
			if(IndexOfWave1 < stepOfWave1) IndexOfWave1	= 0;
		}
		if(IndexOfWave2 !=0)
		{
			IndexOfWave2 += stepOfWave2;
			if(IndexOfWave2 < stepOfWave2) IndexOfWave2	= 0;
		}
		if(IndexOfWave1 == 0 && IndexOfWave2 == 0) AlarmVolume	= 0;
	}
	else
	{
		IndexOfWave1 += stepOfWave1;
		IndexOfWave2 += stepOfWave2;
	}
	
}

//タイマー0オーバーフロー割込み
//ISR(TIMER0_OVF_vect)
//{
//}

void delay_ms( int time )
{
	// 指定ms分ループ
	while( time-- ){
		_delay_ms( 1 );
	}
}

int main( void )
{
	unsigned int	i	= 0, j	= 0,k	= 0;
	unsigned int	timeOfClose	= 0;
	
	Volume = 0;
	//入出力設定
	DDRB	= 0b00000000;
	DDRD	= 0b00000000;
	PORTB	= 0b00000000;
	PORTD	= 0b00000000;
	//警報灯出力
	FLASHLIGHT_L_DDR	= 1;
	FLASHLIGHT_R_DDR	= 1;
	//遮断機出力
	CLOSEGATE_DDR	= 1;
	OPENGATE_DDR	= 1;
	
	//s88出力
	S88OUT_L_DDR	= 1;
	S88OUT_R_DDR	= 1;
	
	//スイッチ入力
	CLOSE_SWITCH_DDR	= 0;
	CLOSE_SWITCH_PORT	= 1;	// プルアップ
	
	CLOSE_DCC_DDR	= 0;	//DCC入力
	SENSOR_L_DDR	= 0;	//センサー入力
	SENSOR_R_DDR	= 0;
	
	// Timer2設定	サウンド用
	DDRD	|= (1 << PD3);	//PD3(OC2B)を出力に設定
	TCCR2A	= 0b00100011;	// 8bit高速PWM動作 比較A出力選択でOC2Bへの非反転動作
	TCCR2B	= 0b00000001;	// タイマ/カウンタ2入力クロック前置分周なし
	TIMSK2	|= (1 << TOIE2);  /* Timer2 overflow int enable */
	
	//Timer1設定	センサー用
	TCCR1A	= 0b00000000;	// 通常動作
	TCCR1B	= 0b00000100;	// クロック前置分周1/256
	
	
	//Timer0設定	警報灯周期用
	TCCR0A	= 0b00000000;	// 通常動作
	TCCR0B	= 0b00000101;	// クロック前置分周1/1024
	//TCNT0	= 0;
	//TIMSK0	|= (1 << TOIE0);  /* Timer0 overflow int enable */
	
	//OCR0A	= 0;
	sei() ;			//割り込みを許可

	DirVolume	= 2;
	while(1){
		
		//警報機の鳴動を止める
		if(			CLOSE_SWITCH_PIN == 0 &&
					bit_is_set(TIFR1,TOV1) &&
					CLOSE_DCC_PIN	== 0)	//入力ピンチェック
		{
			timeOfClose	= 0;	//遮断機の経過時間を元に戻す
			suspend();
		}
		
		check_sensor();
		
		//Timer0オーバーフローによる警報灯周期設定
		if(bit_is_set(TIFR0,TOV0))
		{
			TIFR0	= _BV(TOV0);	//オーバーフローフラグを戻す。
			if(i++ == 4)
			{
				j++;
				k++;
				i	= 0;
				if(timeOfClose	< 20)	timeOfClose++;
			}
		}
		//警報音開始を設定
		if(j == 3)
		{
			DirVolume	= 2;
			j = 0;
		}
		//警報灯の点滅
		if(k ==4)
		{
			if(FLASHLIGHT_R_PORT	== 1)	//一瞬だけ両方の警報灯を光らせる
			FLASHLIGHT_L_PORT	= 0;
			else
			FLASHLIGHT_R_PORT	= 1;
		}
		if(k ==8)
		{
			if(FLASHLIGHT_L_PORT	== 1)//一瞬だけ両方の警報灯を光らせる
			{
				FLASHLIGHT_R_PORT	= 0;
				k	= 0;
			}
			else
			FLASHLIGHT_L_PORT = 1;
		}
		//遮断機の降下
		if(timeOfClose	== 20)
		{
			CLOSEGATE_PORT	= 1;
			timeOfClose	= 21;
		}
	}

	return 0;
}

int suspend( void )
{
	
	DirVolume	= 0;		//消音する。
	
	//遮断機を下げる信号を停止
	CLOSEGATE_PORT	= 0;
	
	//警報灯を消灯する
	FLASHLIGHT_L_PORT	= 1;
	FLASHLIGHT_R_PORT	= 1;
	
	_delay_ms(400);	//チャタリング対策で時間待ち
	
	OPENGATE_PORT	= 1;	//遮断機を上げる
	
	//スピーカーを止める
	OCR2B	= 0;

	while(		CLOSE_SWITCH_PIN == 0 &&
				bit_is_set(TIFR1,TOV1) &&
				CLOSE_DCC_PIN	== 0)	//入力ピンチェック
	{
		check_sensor();
	}
	_delay_ms(200);		//チャタリング対策で時間待ち	
	OPENGATE_PORT	= 0;	//遮断機の降下信号を停止
	DirVolume	= 1;		//音を鳴らさせる
	return 0;
}

void check_sensor( void )
{
	static unsigned int	SensorDirection;	//最初に列車が触ったセンサの方向
	//タイマーがオーバー(センサが非稼働)時に初期化処理
	if(bit_is_set(TIFR1,TOV1))
	{
		SensorDirection	= 0;
		S88OUT_L_PORT	= 0;
		S88OUT_R_PORT	= 0;
	}
	
//センサー入力を調べ，Timer1カウンターを設定する。
	if(SENSOR_L_PIN == 0 || SENSOR_R_PIN == 0)
	{
		TIFR1	= _BV(TOV1);	//オーバーフローフラグリセット
		TCNT1 = 0;
		
		if(SENSOR_L_PIN == 0)
		{
			//先にセンサRを通過していた時，s88出力を出す。
			if(SensorDirection	== 2)
				S88OUT_L_PORT	= 1;
			else
				SensorDirection	= 1;
		}
		if(SENSOR_R_PIN == 0)
		{
			//先にセンサLを通過していた時，s88出力を出す。
			if(SensorDirection	== 1)
				S88OUT_R_PORT	= 1;
			else
				SensorDirection	= 2;
		}
	}
}