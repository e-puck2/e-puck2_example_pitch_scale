/////////////////////////////////////////////////////////////////////
//
// Simple SOLA algorithm implementation.
//
// Copyright (c) Olli Parviainen 2006 <oparviai@iki.fi>
//
/////////////////////////////////////////////////////////////////////

#include <string.h>
#include "sola.h"

// Time scaling factor, values > 1.0 increase, values < 1.0 decrease tempo
// #define TIME_SCALE      1.30   // 30% faster tempo
#define TIME_SCALE      0.87   // 13% slower tempo
// Processing sequence size (100 msec with 16000Hz samplerate)
#define SEQUENCE        1600
// Overlapping size (20 msec)
#define OVERLAP         320
// Best overlap offset seeking window (15 msec)
#define SEEK_WINDOW     240
// Processing sequence flat mid-section duration
#define FLAT_DURATION   (SEQUENCE - 2 * (OVERLAP))
// Theoretical interval between the processing sequences
#define SEQUENCE_SKIP   ((int)((SEQUENCE - OVERLAP) * (TIME_SCALE)))

// Use cross-correlation function to find best overlapping offset
// where input_prev and input_new match best with each other
int seek_best_overlap(const SAMPLE *input_prev, const SAMPLE *input_new)
{
   int i;
   int bestoffset = 0;
   float bestcorr = -1e30f;
   float temp[OVERLAP];

   // Precalculate overlapping slopes with input_prev
   for (i = 0; i < OVERLAP; i ++)
   {
      temp[i] = (float)(input_prev[i] * i * (OVERLAP - i));
   }

   // Find best overlap offset within [0..SEEK_WINDOW]
   for (i = 0; i < SEEK_WINDOW; i ++)
   {
      int j;
      float crosscorr = 0;

      for (j = 0; j < OVERLAP; j ++)
      {
         crosscorr += (float)input_new[i + j] * temp[j];
      }
      if (crosscorr > bestcorr)
      {
         // found new best offset candidate
         bestcorr = crosscorr;
         bestoffset = i;
      }
   }
   return bestoffset;
}


// Overlap 'input_prev' with 'input_new' by sliding the amplitudes during
// OVERLAP samples. Store result to 'output'.
void overlap(SAMPLE *output, const SAMPLE *input_prev, const SAMPLE *input_new)
{
   int i;

   for (i = 0; i < OVERLAP; i ++)
   {
      output[i] = (input_prev[i] * (OVERLAP - i) + input_new[i] * i) / OVERLAP;
   }
}


// SOLA algorithm. Performs time scaling for sample data given in 'input',
// write result to 'output'. Return number of output samples.
int32_t sola(SAMPLE *output, const SAMPLE *input, int32_t num_in_samples) {
	int32_t num_out_samples = 0;
   const SAMPLE *seq_offset = input;
   const SAMPLE *prev_offset;

   while (num_in_samples > SEQUENCE_SKIP + SEEK_WINDOW)
   {
      // copy flat mid-sequence from current processing sequence to output
      memcpy(output, seq_offset, FLAT_DURATION * sizeof(SAMPLE));
      // calculate a pointer to overlap at end of the processing sequence
      prev_offset = seq_offset + FLAT_DURATION;

      // update input pointer to theoretical next processing sequence begin
      input += SEQUENCE_SKIP - OVERLAP;
      // seek actual best matching offset using cross-correlation
      seq_offset = input + seek_best_overlap(prev_offset, input);

      // do overlapping between previous & new sequence, copy result to output
      overlap(output + FLAT_DURATION, prev_offset, seq_offset);

      // Update input & sequence pointers by overlapping amount
      seq_offset += OVERLAP;
      input  += OVERLAP;

      // Update output pointer & sample counters
      output += SEQUENCE - OVERLAP;
      num_out_samples += SEQUENCE - OVERLAP;
      num_in_samples -= SEQUENCE_SKIP;
   }

   return num_out_samples;
}

