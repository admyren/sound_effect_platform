/*
 * filter.c
 *
 *  Created on: 30 nov. 2018
 *      Author: Myren
 */

#include "dsp.h"
#include "drivers.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static float sinc(float x)
{
	if(x == 0)
	{
		return 1.0;
	}
	else
	{
		return sin(M_PI*x)/(M_PI*x);
	}
}

// Calculates coefficients for an FIR filter
void design_FIR(Filter_TypeDef* hFilter)
{
	uint16_t N = hFilter->order;
	hFilter->FIR_instance->order = hFilter->order;
	hFilter->topology = hFilter->topologyBuff;
	float w_cl = M_PI*(float)hFilter->cutOff[0]/(SAMPLING_FREQUENCY/2);
	float w_cu = M_PI*(float)hFilter->cutOff[1]/(SAMPLING_FREQUENCY/2);
	float n;
	if(!(N%2))
	{
		n = -(float)(N/2.0) + 0.5;
	}
	else
	{
		n = -floor(N/2.0);
	}

	if(hFilter->type == LP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (w_cl/M_PI)*sinc((w_cl*n)/M_PI);
			n = n + 1;
		}

	}
	else if(hFilter->type == HP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (sinc(n) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI));
			n = n + 1;
		}
	}
	else if(hFilter->type == BP)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = ((w_cu/M_PI)*sinc((w_cu*n)/M_PI) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI));
			n = n + 1;
		}
	}
	else if(hFilter->type == BS)
	{
		uint16_t k;
		for(k=0; k<N; k++)
		{
			hFilter->FIR_instance->h[k] = (sinc(n) - (((w_cu/M_PI)*sinc((w_cu*n)/M_PI) - (w_cl/M_PI)*sinc((w_cl*n)/M_PI))));
			n = n + 1;
		}
	}
}

// Calculates coefficients for an IIR filter
void design_IIR(Filter_TypeDef* hFilter)
{
	IIR_TypeDef* hIIR = hFilter->IIR_instance;
	hFilter->topology = hFilter->topologyBuff;
	hIIR->order = hFilter->order;
	hIIR->sNs = hFilter->order/2;
	hIIR->fNs = hFilter->order%2;
	float w_cl = 2*M_PI*(float)hFilter->cutOff[0];///(SAMPLING_FREQUENCY/2);
	float w_cu = 2*M_PI*(float)hFilter->cutOff[1];///(SAMPLING_FREQUENCY/2);

	// Storage for continuous time coefficients
	float as_s[3], bs_s[3], af_s[2], bf_s[2];

	as_s[0] = 0;
	as_s[1] = 0;
	as_s[2] = 0;
	bs_s[0] = 0;
	bs_s[1] = 0;
	bs_s[2] = 0;
	af_s[0] = 0;
	af_s[1] = 0;
	bf_s[0] = 0;
	bf_s[1] = 0;

	// Storage for discrete time coefficients
	float as_z[3], bs_z[3], af_z[2], bf_z[2];
	//float K = 2*SAMPLING_FREQUENCY;//15800;//w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
	float K = 0;//w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));

	if(hFilter->type == LP)
	{
		K = w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
		if(hIIR->sNs > 0)
		{
			bs_s[0] = 0; bs_s[1] = 0; bs_s[2] = w_cl*w_cl;
			as_s[0] = 1; as_s[1] = M_SQRT2*w_cl; as_s[2] = w_cl*w_cl;
		}
		if(hIIR->fNs > 0)
		{
			bf_s[0] = 0; bf_s[1] = w_cl;
			af_s[0] = 1; af_s[1] = w_cl;
		}
	}
	else if(hFilter->type == HP)
	{
		K = w_cl/(tan((w_cl/SAMPLING_FREQUENCY)/(2.0)));
		if(hIIR->sNs > 0)
		{
			bs_s[0] = 1; bs_s[1] = 0; bs_s[2] = 0;
			as_s[0] = 1; as_s[1] = M_SQRT2*w_cl; as_s[2] = w_cl*w_cl;
		}
		if(hIIR->fNs > 0)
		{
			bf_s[0] = 1; bf_s[1] = 0;
			af_s[0] = 1; af_s[1] = w_cl;
		}
	}
	else if(hFilter->type == BP)
	{
		// Center frequency
		float w_cf = (w_cl+w_cu)/2;
		K = w_cf/(tan((w_cf/SAMPLING_FREQUENCY)/(2.0)));
		bs_s[0] = 0; bs_s[1] = w_cu-w_cl; bs_s[2] = 0;
		as_s[0] = 1; as_s[1] = w_cu-w_cl; as_s[2] = w_cl*w_cu;

		hIIR->sNs = hFilter->order;
		hIIR->fNs = 0;


	}
	else if(hFilter->type == BS)
	{
		// Center frequency
		float w_cf = (w_cl+w_cu)/2;
		K = w_cf/(tan((w_cf/SAMPLING_FREQUENCY)/(2.0)));
		bs_s[0] = 1; bs_s[1] = 0; bs_s[2] = w_cl*w_cu;
		as_s[0] = 1; as_s[1] = w_cu-w_cl; as_s[2] = w_cl*w_cu;

		hIIR->sNs = hFilter->order;
		hIIR->fNs = 0;
	}

	float den = as_s[0]*K*K + as_s[1]*K + as_s[2];
	as_z[0] = 1.0;
	as_z[1] = (2*as_s[2] - 2*as_s[0]*K*K)/den;
	as_z[2] = (as_s[0]*K*K - as_s[1]*K + as_s[2])/den;

	bs_z[0] = (bs_s[0]*K*K + bs_s[1]*K + bs_s[2])/den;
	bs_z[1] = (2*bs_s[2] - 2*bs_s[0]*K*K)/den;
	bs_z[2] = (bs_s[0]*K*K - bs_s[1]*K + bs_s[2])/den;

	den = af_s[0]*K + af_s[1];

	af_z[0] = 1.0;
	af_z[1] = (af_s[1] - af_s[0]*K)/den;

	bf_z[0] = (bf_s[0]*K + bf_s[1])/den;
	bf_z[1] = (bf_s[1] - bf_s[0]*K)/den;

	hIIR->sCoeffs[0] = as_z[1]; 	// a_1
	hIIR->sCoeffs[1] = as_z[2];		// a_2
	hIIR->sCoeffs[2] = bs_z[2];		// b_2
	hIIR->sCoeffs[3] = bs_z[0];		// b_0
	hIIR->sCoeffs[4] = bs_z[1];		// b_1

	hIIR->fCoeffs[0] = af_z[1];		// a_1
	hIIR->fCoeffs[1] = bf_z[1];		// b_1
	hIIR->fCoeffs[2] = bf_z[0];		// b_0
	// -u _printf_float
// used for debug
#if 0
	char str[40];

	sprintf(str, "%f  ", hIIR->sCoeffs[0]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[1]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[3]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->sCoeffs[4]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  \n", hIIR->sCoeffs[2]);
	UART_Transmit(UART4, (uint8_t*)str, 0);

	sprintf(str, "%f  ", hIIR->fCoeffs[0]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->fCoeffs[2]);
	UART_Transmit(UART4, (uint8_t*)str, 0);
	sprintf(str, "%f  ", hIIR->fCoeffs[1]);
	UART_Transmit(UART4, (uint8_t*)str, 0);


	UART_Transmit(UART4, (uint8_t*)"\n\n", 0);

	sprintf(str, "%f rad/s SOS: %d FOS: %d\n", w_cl, hIIR->sNs, hIIR->fNs);
	UART_Transmit(UART4, (uint8_t*)str, 0);
#endif


}



// Execute a filter channel
void exec_Filters(Ch_TypeDef* hCh)
{
	uint8_t i,j = 0;
	Filter_TypeDef* hFilters = hCh->filters;

	// If channel is not active, copy channel source buffer to destination buffer
	if(hCh->active_flag == NOT_ACTIVE)
	{
		for(i=0; i<BUFFER_SIZE/2; i++)
		{
			hCh->pDst[i] = hCh->pSrc[i];
		}
	}
	else
	{
		for(i=0; i<MAX_FILTERS; i++)
		{
			// Execute filter if it is used
			if(hFilters->used_flag == IS_USED)
			{
				if(hFilters->topology == FIR)
				{
					FIR_Filter(hFilters->FIR_instance, BUFFER_SIZE/2);
				}
				else if(hFilters->topology == IIR)
				{
					IIR_Filter(hFilters->IIR_instance, BUFFER_SIZE/2);

				}
			}
			// Else, copy it to the next buffer
			else if(hFilters->used_flag == NOT_USED)
			{
				for(j=0; j<BUFFER_SIZE/2; j++)
				{
					hFilters->pDst[j] = hFilters->pSrc[j];
				}
			}

		}
	}
}

// Sets the source and destination buffers of a sum node
void init_Sum(SUM_TypeDef* hsum, float* outBuff)
{
	hsum->pSrc1 = ch1_1fBuff;
	hsum->pSrc2 = ch1_2fBuff;
	hsum->pSrc3 = ch1_3fBuff;
	hsum->pDst = outBuff;
}

// This function could be optimized
void exec_Sum(SUM_TypeDef* hsum)
{
	for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
	{
		hsum->pDst[i] = (hsum->ChActive[0]*hsum->pSrc1[i] +
				         hsum->ChActive[1]*hsum->pSrc2[i] +
						 hsum->ChActive[2]*hsum->pSrc3[i]);
		if(hsum->pDst[i] > 2047)
		{
			hsum->pDst[i] = 2047;
		}
		else if(hsum->pDst[i] < -2047)
		{
			hsum->pDst[i] = -2047;
		}
	}
}

// Cascaded IIR filter
void IIR_Filter(IIR_TypeDef* hIIR, uint16_t blkSize)
{
	uint16_t i,n,l,s;
	float temp16;
	float w_0,temp32;

	s = hIIR->sNs*2; // Set up circular buffer w[]

	for (l=0,n=0; n<blkSize; n++) // IIR filtering
	{
		w_0 = hIIR->pSrc[n]; // Scale input to prevent overflow
		// Calculate second order sections
		for (i=0; i<hIIR->sNs; i++)
		{
			// temp32 = a_1*v[n-1]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[0];
			l=(l+hIIR->sNs)%s;
			w_0 = w_0 - temp32;

			// temp32 = a_2*v[n-2]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[1];
			// w_0 now holds v[n]
			w_0 = w_0 - temp32;

			// temp16 = v[n-2]
			temp16 = hIIR->sBuff[l];
			// Save v[n]
			hIIR->sBuff[l] = (w_0); // Save in Q15 format
			// w_0 = b_2*v[n-2]
			w_0 = temp16*hIIR->sCoeffs[2];

			// temp32 = b_0*v[n]
			temp32 = (int32_t)(hIIR->sBuff[l])*hIIR->sCoeffs[3];

			l=(l+hIIR->sNs)%s;
			w_0 = w_0 + temp32;
			// temp32 = b_1*v[n-1]
			temp32 = (hIIR->sBuff[l])*hIIR->sCoeffs[4];

			l=(l+1)%s;
			w_0 = w_0 + temp32;

		}
		// Calculate first order section
		if(hIIR->fNs == 1)
		{
			// temp32 = a_1*v[n-1]
			w_0 -= *hIIR->fBuff*hIIR->fCoeffs[0];
			// w_0 now holds v[n]
			//w_0 -= temp32;

			// temp16 = v[n-1]
			temp16 = *hIIR->fBuff;
			// Save v[n]
			*hIIR->fBuff = w_0; // Save in Q15 format
			// w_0 = b_1*v[n-1]
			w_0 = temp16*hIIR->fCoeffs[2];
			//w_0 >>= 1;

			// temp32 = b_0*v[n]
			w_0 += *hIIR->fBuff*hIIR->fCoeffs[1];
			//w_0 += temp32;
			//w_0 <<= 1;
		}
		hIIR->pDst[n] = w_0;
	}

}

void FIR_Filter(FIR_TypeDef* hFIR, uint16_t blkSize)
{
#if 1
	int16_t i,j,k;
	float sum;
	float *c;
	float *x = hFIR->pSrc;
	float *y = hFIR->pDst;
	k = hFIR->index;;
	for (j=0; j<blkSize; j++) // Block processing
	{
		hFIR->state_buffer[k] = *x++; // Get current data to delay line

		c = hFIR->h;
		for (sum=0, i=0; i<hFIR->order; i++) // FIR filter processing
		{
			sum += *c++*hFIR->state_buffer[k++];
			//sum += vmull_n_s16(*c++, w[k++]);
			if (k == hFIR->order) // Simulate circular buffer
				k = 0;
		}
		*y++ = sum; // Save filter output
		//hFIR->pDst++;
		if (k-- <= 0) // Update index for next time
			k = hFIR->order-1;
	}
	hFIR->index = k; // Update circular buffer index
#else
	for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
	{
		hFIR->pDst[i] = hFIR->pSrc[i];
	}

#endif

}

void init_Filter(Filter_TypeDef *hFilter, uint16_t buffer_size, float *pSrc, float *pDst)
{
	hFilter->topology = FIR;
	hFilter->topologyBuff = FIR;
	hFilter->order = 10;
	hFilter->cutOff[0] = 4000;
	hFilter->cutOff[1] = 8000;
	hFilter->pSrc = pSrc;
	hFilter->pDst = pDst;
	hFilter->type = LP;


	hFilter->FIR_instance = malloc(sizeof(FIR_TypeDef));
	hFilter->FIR_instance->h = malloc(MAX_ORDER_FIR*sizeof(float));
	hFilter->FIR_instance->state_buffer = malloc((MAX_ORDER_FIR+buffer_size/2)*sizeof(float));
	hFilter->FIR_instance->order = hFilter->order;
	hFilter->FIR_instance->index = 0;
	hFilter->FIR_instance->pSrc = hFilter->pSrc;
	hFilter->FIR_instance->pDst = hFilter->pDst;

}





