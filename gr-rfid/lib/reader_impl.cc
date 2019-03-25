/* -*- c++ -*- */
/* 
 * Copyright 2015 <Nikos Kargas (nkargas@isc.tuc.gr)>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "reader_impl.h"
#include "rfid/global_vars.h"
#include "crc_t.h"
#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <string.h>

namespace gr {
  namespace rfid {

    reader::sptr
    reader::make(int sample_rate, int dac_rate, bool select, const std::string &select_mask)
    {
      return gnuradio::get_initial_sptr
        (new reader_impl(sample_rate,dac_rate,select,select_mask));
    }

    /*
     * The private constructor
     */
    reader_impl::reader_impl(int sample_rate, int dac_rate, bool select, const std::string &select_mask)
      : gr::block("reader",
              gr::io_signature::make( 1, 1, sizeof(float)),
              gr::io_signature::make( 1, 1, sizeof(float)))
    {

      GR_LOG_INFO(d_logger, "Block initialized");

      sample_d = 1.0/dac_rate * pow(10,6);

      // Number of samples for transmitting

      n_data0_s = 2 * PW_D / sample_d;
      n_data1_s = 4 * PW_D / sample_d;
      n_pw_s    = PW_D    / sample_d;
      n_cw_s    = CW_D    / sample_d;
      n_delim_s = DELIM_D / sample_d;
      n_trcal_s = TRCAL_D / sample_d;

      GR_LOG_INFO(d_logger, "Number of samples data 0 : " << n_data0_s);
      GR_LOG_INFO(d_logger, "Number of samples data 1 : " << n_data1_s);
      GR_LOG_INFO(d_logger, "Number of samples cw : "     << n_cw_s);
      GR_LOG_INFO(d_logger, "Number of samples delim : "  << n_delim_s);
      GR_LOG_INFO(d_logger, "Number of slots : "          << std::pow(2,FIXED_Q));

      // CW waveforms of different sizes
      n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16
      n_cwack_s     = (3*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag
      n_p_down_s    = (P_DOWN_D)/sample_d;
      n_cwselect_s  = T4_D/sample_d;                   //SELECT
      n_cwsettle_s  = TS_D/sample_d;                   //SETTLE

      p_down.resize(n_p_down_s);        // Power down samples
      cw_query.resize(n_cwquery_s);     // Sent after query/query rep
      cw_ack.resize(n_cwack_s);         // Sent after ack
      cw_select.resize(n_cwselect_s);   // Sent after select
      cw_settle.resize(n_cwsettle_s);   // Sent before first Interrogator Command (TAG wakeup time)

      std::fill_n(cw_query.begin(), cw_query.size(), 1);
      std::fill_n(cw_ack.begin(), cw_ack.size(), 1);
      std::fill_n(cw_select.begin(), cw_select.size(), 1);
      std::fill_n(cw_settle.begin(), cw_settle.size(), 1);

      GR_LOG_INFO(d_logger, "Carrier wave after a query transmission in samples : "     << n_cwquery_s);
      GR_LOG_INFO(d_logger, "Carrier wave after ACK transmission in samples : "        << n_cwack_s);
      GR_LOG_INFO(d_logger, "Carrier wave after a select transmission in samples : "     << n_cwselect_s);
      GR_LOG_INFO(d_logger, "Carrier wave before interrogator transmission in samples : "     << n_cwsettle_s);

      // Construct vectors (resize() default initialization is zero)
      data_0.resize(n_data0_s);
      data_1.resize(n_data1_s);
      cw.resize(n_cw_s);
      delim.resize(n_delim_s);
      rtcal.resize(n_data0_s + n_data1_s);
      trcal.resize(n_trcal_s);

      // Fill vectors with data
      std::fill_n(data_0.begin(), data_0.size()/2, 1);
      std::fill_n(data_1.begin(), 3*data_1.size()/4, 1);
      std::fill_n(cw.begin(), cw.size(), 1);
      std::fill_n(rtcal.begin(), rtcal.size() - n_pw_s, 1); // RTcal
      std::fill_n(trcal.begin(), trcal.size() - n_pw_s, 1); // TRcal

      // create preamble
      preamble.insert( preamble.end(), delim.begin(), delim.end() );
      preamble.insert( preamble.end(), data_0.begin(), data_0.end() );
      preamble.insert( preamble.end(), rtcal.begin(), rtcal.end() );
      preamble.insert( preamble.end(), trcal.begin(), trcal.end() );

      // create framesync
      frame_sync.insert( frame_sync.end(), delim.begin() , delim.end() );
      frame_sync.insert( frame_sync.end(), data_0.begin(), data_0.end() );
      frame_sync.insert( frame_sync.end(), rtcal.begin() , rtcal.end() );
      
      // create query rep
      query_rep.insert( query_rep.end(), frame_sync.begin(), frame_sync.end());
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
      query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );

      // create nak
      nak.insert( nak.end(), frame_sync.begin(), frame_sync.end());
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_1.begin(), data_1.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );
      nak.insert( nak.end(), data_0.begin(), data_0.end() );

      gen_query_adjust_bits();

      // Adam Laurie
      gen_query_bits(select);
      if(select)
      {
        // add mask to SELECT (empty mask selects all)
        std::vector<float> mask= {};
        for(int i= 0 ; i < select_mask.size() ; i++)
          if(select_mask[i] == '0')
            mask.push_back((float) 0);
          else
            mask.push_back((float) 1);
        gen_select_bits(mask);
      }
    }

    void reader_impl::gen_query_bits(bool select)
    {
      query_bits.resize(0);
      query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);
      query_bits.push_back(DR);
      query_bits.insert(query_bits.end(), &M[0], &M[2]);
      query_bits.push_back(TREXT);
      if(select)
        query_bits.insert(query_bits.end(), &SEL_SL[0], &SEL_SL[2]);
      else
        query_bits.insert(query_bits.end(), &SEL_ALL[0], &SEL_ALL[2]);
      query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
      query_bits.push_back(TARGET);
    
      query_bits.insert(query_bits.end(), &Q_VALUE[FIXED_Q][0], &Q_VALUE[FIXED_Q][4]);
      crc_append(query_bits);
    }


    void reader_impl::gen_ack_bits(const float * in)
    {
      ack_bits.resize(0);
      ack_bits.insert(ack_bits.end(), &ACK_CODE[0], &ACK_CODE[2]);
      ack_bits.insert(ack_bits.end(), &in[0], &in[16]);
    }
  
    void reader_impl::gen_query_adjust_bits()
    {
      query_adjust_bits.resize(0);
      query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
      query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
      query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[1][0], &Q_UPDN[1][3]);
    }


    // Adam Laurie
    void reader_impl::gen_select_bits(std::vector<float> & mask)
    {
      select_bits.resize(0);
      select_bits.insert(select_bits.end(), &SELECT_CODE[0], &SELECT_CODE[4]);
      select_bits.insert(select_bits.end(), &SELECT_TARGET[0], &SELECT_TARGET[3]);
      select_bits.insert(select_bits.end(), &SELECT_ACTION[0], &SELECT_ACTION[3]);
      select_bits.insert(select_bits.end(), &SELECT_MEM[0], &SELECT_MEM[2]);
      select_bits.insert(select_bits.end(), &SELECT_POINT[0], &SELECT_POINT[8]);
      // set mask size
      for(int i= 7 ; i >= 0 ; i--)
        select_bits.push_back((float) ((mask.size() >> i) & 0x01));
      // set mask
      if(mask.size() > 0)
        select_bits.insert(select_bits.end(), mask.begin(), mask.end());
      select_bits.push_back(SELECT_TRUNC);

      crc_16_append(select_bits);
    }

    /*
     * Our virtual destructor.
     */
    reader_impl::~reader_impl()
    {

    }

    void reader_impl::print_results()
    {
      std::cout << "\n --------------------------" << std::endl;
      std::cout << "| Number of queries/queryreps sent : " << reader_state->reader_stats.n_queries_sent - 1 << std::endl;
      std::cout << "| Current Inventory round : "          << reader_state->reader_stats.cur_inventory_round << std::endl;
      std::cout << " --------------------------"            << std::endl;

      std::cout << "| Correctly decoded EPC : "  <<  reader_state->reader_stats.n_epc_correct     << std::endl;
      std::cout << "| Number of unique tags : "  <<  reader_state->reader_stats.tag_reads.size() << std::endl;

      std::map<int,int>::iterator it;

      for(it = reader_state->reader_stats.tag_reads.begin(); it != reader_state->reader_stats.tag_reads.end(); it++) 
      {
        std::cout << std::hex <<  "| Tag ID : " << it->first << "  ";
        std::cout << "Num of reads : " << std::dec << it->second << std::endl;
      }

      std::cout << " --------------------------" << std::endl;
      // Adam Laurie
      // Force re-start
      reader_state->gen2_logic_status= START;
    }

    void
    reader_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = 0;
    }

    int
    reader_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {

      const float *in = (const float *) input_items[0];
      float *out =  (float*) output_items[0];
      std::vector<float> out_message; 
      int n_output;
      int consumed = 0;
      int written = 0;

      consumed = ninput_items[0];
  
      switch (reader_state->gen2_logic_status)
      {
        case START:
          GR_LOG_INFO(d_debug_logger, "START");

          memcpy(&out[written], &cw_settle[0], sizeof(float) * cw_settle.size() );
          written += cw_settle.size();
          // Adam Laurie
          if(select)
            reader_state->gen2_logic_status = SEND_SELECT;
          else
            reader_state->gen2_logic_status = SEND_QUERY;
          break;

        case POWER_DOWN:
          GR_LOG_INFO(d_debug_logger, "POWER DOWN");
          memcpy(&out[written], &p_down[0], sizeof(float) * p_down.size() );
          written += p_down.size();
          reader_state->gen2_logic_status = START;    
          break;

        case SEND_NAK_QR:
          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY_REP;
          break;

        case SEND_NAK_Q:
          GR_LOG_INFO(d_debug_logger, "SEND NAK");
          memcpy(&out[written], &nak[0], sizeof(float) * nak.size() );
          written += nak.size();
          memcpy(&out[written], &cw[0], sizeof(float) * cw.size() );
          written+=cw.size();
          reader_state->gen2_logic_status = SEND_QUERY;
          break;

        // Adam Laurie
        case SEND_SELECT:
          GR_LOG_INFO(d_debug_logger, "SELECT");
          //std::cout << "SELECT" << std::endl;

          memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
          written+=frame_sync.size();

          for(int i = 0; i < select_bits.size(); i++)
          {
            if(select_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }

          // gap
          memcpy(&out[written], &cw_select[0], sizeof(float) * cw_select.size() );
          written += cw_select.size();

          reader_state->gen2_logic_status = SEND_QUERY;
          break;

        case SEND_QUERY:

          /*if (reader_state->reader_stats.n_queries_sent % 500 == 0)
          {
            std::cout << "Running " << std::endl;
          }*/

          GR_LOG_INFO(d_debug_logger, "QUERY");
          GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);

          reader_state->reader_stats.n_queries_sent +=1;  
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;

          memcpy(&out[written], &preamble[0], sizeof(float) * preamble.size() );
          written+=preamble.size();
   
          for(int i = 0; i < query_bits.size(); i++)
          {
            if(query_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }
          // Send CW for RN16
          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size() );
          written+=cw_query.size();

          // Return to IDLE
          reader_state->gen2_logic_status = IDLE;      
          break;

        case SEND_ACK:
          GR_LOG_INFO(d_debug_logger, "SEND ACK");
          if (ninput_items[0] == RN16_BITS - 1)
          {
            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_EPC;
            reader_state->gate_status    = GATE_SEEK_EPC;

            gen_ack_bits(in);
          
            // Send FrameSync
            memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
            written += frame_sync.size();

            for(int i = 0; i < ack_bits.size(); i++)
            {
              if(ack_bits[i] == 1)
              {
                memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
                written += data_1.size();
              }
              else  
              {
                memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
                written += data_0.size();
              }
            }
             consumed = ninput_items[0];
            reader_state->gen2_logic_status = SEND_CW; 
          }
          break;

        case SEND_CW:
          GR_LOG_INFO(d_debug_logger, "SEND CW");
          memcpy(&out[written], &cw_ack[0], sizeof(float) * cw_ack.size() );
          written += cw_ack.size();
          reader_state->gen2_logic_status = IDLE;      // Return to IDLE
          break;

        case SEND_QUERY_REP:
          GR_LOG_INFO(d_debug_logger, "SEND QUERY_REP");
          GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &query_rep[0], sizeof(float) * query_rep.size() );
          written += query_rep.size();

          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size());
          written+=cw_query.size();

          reader_state->gen2_logic_status = IDLE;    // Return to IDLE
          break;
      
        case SEND_QUERY_ADJUST:
          GR_LOG_INFO(d_debug_logger, "SEND QUERY_ADJUST");
          // Controls the other two blocks
          reader_state->decoder_status = DECODER_DECODE_RN16;
          reader_state->gate_status    = GATE_SEEK_RN16;
          reader_state->reader_stats.n_queries_sent +=1;  

          memcpy(&out[written], &frame_sync[0], sizeof(float) * frame_sync.size() );
          written += frame_sync.size();

          for(int i = 0; i < query_adjust_bits.size(); i++)
          {
            if(query_adjust_bits[i] == 1)
            {
              memcpy(&out[written], &data_1[0], sizeof(float) * data_1.size() );
              written+=data_1.size();
            }
            else
            {
              memcpy(&out[written], &data_0[0], sizeof(float) * data_0.size() );
              written+=data_0.size();
            }
          }
          memcpy(&out[written], &cw_query[0], sizeof(float) * cw_query.size());
          written+=cw_query.size();
          reader_state->gen2_logic_status = IDLE;    // Return to IDLE
          break;

        default:
          // IDLE
          break;
      }
      consume_each (consumed);
      return  written;
    }

    /* Function adapted from https://www.cgran.org/wiki/Gen2 */
    // 5 bit CRC for QUERY
    void reader_impl::crc_append(std::vector<float> & q)
    {
       int crc[] = {1,0,0,1,0};

      for(int i = 0; i < 17; i++)
      {
        int tmp[] = {0,0,0,0,0};
        tmp[4] = crc[3];
        if(crc[4] == 1)
        {
          if (q[i] == 1)
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
          else
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
        }
        else
        {
          if (q[i] == 1)
          {
            tmp[0] = 1;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            if(crc[2] == 1)
            {
              tmp[3] = 0;
            }
            else
            {
              tmp[3] = 1;
            }
          }
          else
          {
            tmp[0] = 0;
            tmp[1] = crc[0];
            tmp[2] = crc[1];
            tmp[3] = crc[2];
          }
        }
        memcpy(crc, tmp, 5*sizeof(float));
      }
      for (int i = 4; i >= 0; i--)
        q.push_back((float) crc[i]);
    }

    // Adam Laurie
    // 16 bit crc for SELECT
    // test with input of '000000000100000000010' should be 0xC797
    // warning: This code assumes no command is less then 16 bits.
    void reader_impl::crc_16_append(std::vector<float> & q)
    {
      int num_bytes;
      uint16_t crc;
      int offset= 0;

      // make a copy of the input with leading 0s for byte alignment
      // CRC algo can't use preset for non byte aligned/length bitstream so we also need to manually
      // XOR the 1st 16 bits of the input with the preset (0xffff)
      if(q.size() % 8)
        offset= (8 - (q.size() % 8));
      int *qbuf= (int *) malloc((q.size() + offset) * sizeof(int));
      if(!qbuf)
      {
        std::cout << "Memory Allocation Failed" << std::endl;
        exit(1);
      }

      for(int i= 0 ; i < offset ; i++)
        qbuf[i]= 0x00;
      for(int i= 0 ; i < 16 ; i++)
        qbuf[i + offset]= (int) q[i] ^ 0x01;
      for(int i= 16 ; i < q.size() ; i++)
        qbuf[i + offset]= (int) q[i];

      // copy to bytes
      num_bytes= q.size() / 8;
      if(q.size() % 8)
        num_bytes += 1;
      unsigned char *buf= (unsigned char*) malloc(num_bytes * sizeof(unsigned char));
      if(!buf)
      {
        std::cout << "Memory Allocation Failed" << std::endl;
        exit(1);
      }
      memset(buf, 0x00, num_bytes);
      for(int i= 0 ; i < num_bytes ; i++)
        for(int j= 0 ; j < 8 ; ++j)
          buf[i] += (unsigned char) (qbuf[i * 8 + j]) << 7 - j;

      // Create a CRC-16/EPC function but with 0x0000 preset as we've already applied it. 
      CRC_t<16, 0x1021, 0x0000, false, false, 0xffff> crc_epc;

      crc= crc_epc.get_crc(buf, num_bytes);

      // push CRC bits back to source buffer
      for(int i= 15 ; i >= 0 ; i--)
          q.push_back((float) ((crc >> i) & 0x01));
      free(qbuf);
      free(buf);
    }
  } /* namespace rfid */
} /* namespace gr */

