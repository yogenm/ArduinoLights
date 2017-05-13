/*
fht_adc.pde
guest openmusiclabs.com 9.5.12
example sketch for testing the fht library.
it takes in data on ADC0 (Analog0) and processes them
with the fht. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.
*/

#define USE_LED_MATRIX 1
//#define ADC_FREE_RUNNING 1
//#define SEND_FFT_TO_PLOTTER

#define AUDIO_READ_PIN 0

#ifdef USE_LED_MATRIX

#include <Adafruit_NeoPixel.h>
#define LED_PIN     6

const uint8_t kMatrixWidth  = 15;
const uint8_t kMatrixHeight = 10;
const bool    kMatrixSerpentineLayout = true;

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
#define MAX_DIMENSION ((kMatrixWidth>kMatrixHeight) ? kMatrixWidth : kMatrixHeight)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

struct ColorRange
{
	uint8_t startR;
	uint8_t startG;
	uint8_t startB;

	int8_t incrementR;
	int8_t incrementG;
	int8_t incrementB;
};

#endif //USE_LED_MATRIX

//#define LIN_OUT 1 
//#define LIN_OUT8 1 
//#define LOG_OUT 1
#define OCTAVE 1
//#define OCT_NORM 1
#define FHT_N 256 // set to 256 point fht
#define PIN_GATE_IN 2


#include <FHT.h> // include the library

void setup() {

#ifdef SEND_FFT_TO_PLOTTER 
  Serial.begin(115200); // use the serial port
#endif
#ifdef ADC_FREE_RUNNING 
  //TIMSK0 = 0; // turn off timer0 for lower jitter
  ADCSRA = 0xe5; // set the adc to free running mode
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0
#else
  pinMode(AUDIO_READ_PIN, INPUT);
#endif

  pinMode(PIN_GATE_IN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(PIN_GATE_IN), soundISR, CHANGE);

#ifdef USE_LED_MATRIX
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(127);
#endif
}

void zero_fht_buffer()
{
    for (int i = 0 ; i < FHT_N ; i++) 
    {
      fht_input[i] = 0;
    }  
}

#ifdef ADC_FREE_RUNNING
void read_adc()
{
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < FHT_N ; i++) 
    {
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf5; // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      //k -= 0x0200; // form into a signed int
      //k <<= 6; // form into a 16b signed int
      fht_input[i] = k; // put real data into bins
    }
    error out please!
    //DoFFT();
    sei(); 
}
#else
void read_adc()
{
    for (int i = 0 ; i < FHT_N ; i++) 
    {
      fht_input[i] = analogRead(AUDIO_READ_PIN);
    }
}
#endif

void DoFFT()
{
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    
    fht_mag_octave(); // take the output of the fht

    ProcessData();
}

const int samplesToKeep = 10;
const int lastSampleIndex = samplesToKeep - 1;
int data[samplesToKeep][LOG_N] = { 0 };
int baselineData[LOG_N] = { 0 };
int offsetValues[LOG_N] = { 0 };
int movingAverage[LOG_N] = { 0 };
int maxSeen[LOG_N] = { 0 };
int samplesRecorded = 0;
bool gotEnoughSamples = false;

/*int maxSeenUpdated = 0;
int UpdateMaxAfterEveryNSamples = 100;
int updateFrequencyMax = 1000;*/

void ProcessData()
{
	//move all the existing samples back one slot
	for (int sample = 1; sample < samplesToKeep; sample++)
	{
		for (int bin = 0; bin < LOG_N; bin++)
			data[sample - 1][bin] = data[sample][bin];
	}

	//if (gotEnoughSamples)
	{
		for (int bin = 0; bin < LOG_N; bin++)
		{
			//update last slot
			data[lastSampleIndex][bin] = fht_oct_out[bin] - baselineData[bin];


			//update baselineData if necessary
			if (baselineData[bin] > fht_oct_out[bin])
				baselineData[bin] = fht_oct_out[bin];
		}

		//calculate moving average
		for (int bin = 0; bin < LOG_N; bin++)
		{
			float avg = 0;
			for (int sample = 0; sample < samplesToKeep; sample++)
			{
				avg += data[sample][bin];
			}
			movingAverage[bin] = avg / samplesToKeep;
		}

		//calculate max of all the samples
		//update every 'UpdateMaxAfterEveryNSamples' (100) samples
		/*maxSeenUpdated++;
		if (maxSeenUpdated >= UpdateMaxAfterEveryNSamples)
		{
			if (UpdateMaxAfterEveryNSamples < updateFrequencyMax)
				UpdateMaxAfterEveryNSamples += 100;
			maxSeenUpdated = 0;
			for (int bin = 0; bin < LOG_N; bin++)
			{
				int maxValue = data[0][bin];
				for (int sample = 1; sample < samplesToKeep; sample++)
				{
					if (data[sample][bin] > maxValue)
						maxSeen[bin] = maxValue;
				}
			}
		}*/
	}

	if (!gotEnoughSamples)
	{
		samplesRecorded++;
		if (samplesRecorded >= samplesToKeep)
		{
			for (int bin = 0; bin < LOG_N; bin++)
			{
				float avg = 0;
				for (int sample = 0; sample < samplesToKeep; sample++)
				{
					avg += data[sample][bin];
				}
				baselineData[bin] = avg / samplesToKeep;
			}

			gotEnoughSamples = true;
			//maxSeenUpdated = UpdateMaxAfterEveryNSamples;//force maxSeen array to update next time
		}
	}
}

void DumpFFT()
{

#ifdef SEND_FFT_TO_PLOTTER
	
	char buffer[100];
	char *curr = buffer;
	for (int bin = 2; bin < LOG_N; bin++)
	{
		curr += sprintf(curr, "%02d ", maxSeen[bin]);
	}
	Serial.print(buffer);
	Serial.print("  ");

	curr = buffer;
	for (int bin = 2; bin < LOG_N; bin++)
	{
		curr += sprintf(curr, "%02d ", movingAverage[bin]);
	}
	Serial.print(buffer);
	Serial.println("");
#endif

}

void loop() {
  
    zero_fht_buffer();
    //if (digitalRead(PIN_GATE_IN))
	{
		read_adc();
		DoFFT();
		DumpFFT();
#ifdef USE_LED_MATRIX
		Blackout();
		ShowLights();
#endif 
	}
}

#ifdef USE_LED_MATRIX

void Blackout()
{
  for (int i=0; i<NUM_LEDS; i++)
    strip.setPixelColor(i, 0);
  strip.show();
}

void GetColorRange(int bucket, int intervals, struct ColorRange &range)
{
	int8_t endR, endG, endB;
	switch (bucket)
	{
	case 2:
		range.startR = 23;
		range.startG = 255;
		range.startB = 0;
		endR = 232;
		endG = 0;
		endB = 255;
		break;
	case 3:
		range.startR = 255;
		range.startG = 138;
		range.startB = 0;
		endR = 0;
		endG = 117;
		endB = 255;
		break;
	case 4:
		range.startR = 73;
		range.startG = 231;
		range.startB = 108;
		endR = 231;
		endG = 73;
		endB = 196;
		break;
	case 5:
		range.startR = 0;
		range.startG = 148;
		range.startB = 255;
		endR = 255;
		endG = 107;
		endB = 0;
		break;
	case 6:
		break;
	}
	
	range.incrementR = (endR - range.startR) / float(intervals);
	range.incrementG = (endG - range.startG) / float(intervals);
	range.incrementB = (endB - range.startB) / float(intervals);
}

void PaintStrip(int bucket, int mag, int maxMag, int whichRow)
{
	ColorRange colorRange;

	int maxRow = kMatrixWidth * (float(mag) / float(maxMag));
	if (maxRow > kMatrixWidth)
		maxRow = kMatrixWidth;

	GetColorRange(bucket, maxRow, colorRange);

	uint8_t r = colorRange.startR;
	uint8_t g = colorRange.startG;
	uint8_t b = colorRange.startB;

	for (int x = 0; x<maxRow; x++)
	{
		int y = whichRow;
		strip.setPixelColor(XY(x, y), r, g, b);
		strip.setPixelColor(XY(x, y + 1), r, g, b);
		r += colorRange.incrementR;
		g += colorRange.incrementG;
		b += colorRange.incrementB;
	}

}

void ShowLights()
{
	PaintStrip(2, movingAverage[2], 30, 0);
	PaintStrip(3, movingAverage[3], 30, 2);
	PaintStrip(4, movingAverage[4], 30, 4);
	PaintStrip(5, movingAverage[6], 15, 6);
	PaintStrip(6, movingAverage[6] + movingAverage[7], 15, 8);

    strip.show();
}

uint16_t XY(uint8_t x, uint8_t y)
{
	uint16_t i;
	if (kMatrixSerpentineLayout == false)
	{
		i = (y * kMatrixWidth) + x;
	}
	else
	{
		if (y & 0x01)
		{
			// Odd rows run backwards
			uint8_t reverseX = (kMatrixWidth - 1) - x;
			i = (y * kMatrixWidth) + reverseX;
		}
		else
		{
			// Even rows run forwards
			i = (y * kMatrixWidth) + x;
		}
	}
	return i;
}
#endif
