#Developed by: Nikos Kargas 
#Modified by: Adam Laurie <adam@algroup.co.uk> for bladeRF

from gnuradio import gr
from gnuradio import blocks
from gnuradio import filter
from gnuradio import analog
from gnuradio import digital
from gnuradio import qtgui
import osmosdr
import rfid

DEBUG = False

class reader_top_block(gr.top_block):

  # Configure bladeRF source
  def u_source(self):
    # the original project notes that USB latency is an issue and provides a patch for gnuradio
    # to pass data as fast as possible back to the code instead of buffering it.
    # We can simply tune the bladerf buffers to achieve the same effect. Default is 32 buffers of
    # 4K each. We will go to the minimum: 2 buffers of 1K each.
    # https://github.com/ransford/gen2_rfid/blob/master/rfid/README.rfid
    self.source = osmosdr.source( args="numchan=" + str(1) + " " + 'bladerf=0,buffers=2,buflen=1024' )
    self.source.set_sample_rate(self.adc_rate)
    self.source.set_center_freq(self.freq, 0)
    self.source.set_freq_corr(0, 0)
    self.source.set_dc_offset_mode(0, 0)
    self.source.set_iq_balance_mode(0, 0)
    self.source.set_gain_mode(False, 0)
    self.source.set_gain(self.rx_gain, 0)
    self.source.set_antenna('', 0)
    self.source.set_bandwidth(0, 0)

  # Configure bladeRF sink
  def u_sink(self):
    self.sink = osmosdr.sink( args="numchan=" + str(1) + " " + 'bladerf=0,buffers=2,buflen=1024' )
    self.sink.set_sample_rate(self.dac_rate)
    self.sink.set_center_freq(self.freq, 0)
    self.sink.set_freq_corr(0, 0)
    self.sink.set_gain(self.tx_gain, 0)
    self.sink.set_antenna('', 0)
    self.sink.set_bandwidth(0, 0)
    
  def __init__(self):
    gr.top_block.__init__(self)


    #rt = gr.enable_realtime_scheduling() 

    # Adam Laurie
    self.select = True                  # set to True to use SELECT before initial QUERY
    #self.mask = '111'                   # SELECT bit mask (up to 256 bits, e.g. '10100'). empty mask matches all tags.
    self.mask = ''

    ######## Variables #########
    self.dac_rate = 1e6                 # DAC rate 
    self.adc_rate = 100e6/50            # ADC rate (2MS/s complex samples)
    self.decim     = 5                    # Decimation (downsampling factor)
    # min seems to be .3 with max TX gain (60)
    # max is .7
    self.ampl     = 1                  # Output signal amplitude (signal power vary for different RFX900 cards)
    self.freq     = 868e6                # Modulation frequency (can be set between 865.6 - 867.6 or 915 - 921 in UK)
                                         # see https://www.gs1.org/sites/default/files/docs/epc/uhf_regulations.pdf
    #self.freq     = 910e6                # Modulation frequency (can be set between 902-920)
    self.rx_gain   = 6                      # overall gain - libbladerf2.0 will figure it out!
    self.tx_gain   = 60                     # note that a setting of '0' will be ignored!!

    # Each FM0 symbol consists of ADC_RATE/BLF samples (2e6/40e3 = 50 samples)
    # 10 samples per symbol after matched filtering and decimation
    self.num_taps     = [1] * 25 # matched to half symbol period

    ######## File sinks for debugging (1 for each block) #########
    self.file_sink_source         = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/source", False)
    self.file_sink_matched_filter = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/matched_filter", False)
    self.file_sink_gate           = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/gate", False)
    self.file_sink_decoder        = blocks.file_sink(gr.sizeof_gr_complex*1, "../misc/data/decoder", False)
    self.file_sink_reader         = blocks.file_sink(gr.sizeof_float*1,      "../misc/data/reader", False)

    ######## Blocks #########
    self.matched_filter = filter.fir_filter_ccc(self.decim, self.num_taps);
    self.gate            = rfid.gate(int(self.adc_rate/self.decim))
    self.tag_decoder    = rfid.tag_decoder(int(self.adc_rate/self.decim))
    self.reader          = rfid.reader(int(self.adc_rate/self.decim),int(self.dac_rate),self.select,self.mask)
    self.amp              = blocks.multiply_const_ff(self.ampl)
    self.to_complex      = blocks.float_to_complex()

    if (DEBUG == False) : # Real Time Execution

      # USRP blocks
      self.u_source()
      self.u_sink()

      ######## Connections #########
      self.connect(self.source,  self.matched_filter)
      self.connect(self.matched_filter, self.gate)

      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect(self.reader, self.amp)
      self.connect(self.amp, self.to_complex)

      # addy - comment out and sink to /dev/null for sniff mode
      self.connect(self.to_complex, self.sink)
      #self.file_sink                  = blocks.file_sink(gr.sizeof_gr_complex*1,   "/dev/null", False)
      #self.connect(self.to_complex, self.file_sink)

      #File sinks for logging (Remove comments to log data)
      self.connect(self.source, self.file_sink_source)

    else :  # Offline Data
      self.file_source               = blocks.file_source(gr.sizeof_gr_complex*1, "../misc/data/file_source_test",False)   ## instead of uhd.usrp_source
      self.file_sink                  = blocks.file_sink(gr.sizeof_gr_complex*1,   "../misc/data/file_sink", False)     ## instead of uhd.usrp_sink
 
      ######## Connections ######### 
      self.connect(self.file_source, self.matched_filter)
      self.connect(self.matched_filter, self.gate)
      self.connect(self.gate, self.tag_decoder)
      self.connect((self.tag_decoder,0), self.reader)
      self.connect(self.reader, self.amp)
      self.connect(self.amp, self.to_complex)
      self.connect(self.to_complex, self.file_sink)
    
    #File sinks for logging 
    #self.connect(self.gate, self.file_sink_gate)
    self.connect((self.tag_decoder,1), self.file_sink_decoder) # (Do not comment this line)
    #self.connect(self.file_sink_reader, self.file_sink_reader)
    #self.connect(self.matched_filter, self.file_sink_matched_filter)

if __name__ == '__main__':

  main_block = reader_top_block()
  main_block.start()

  while(1):
    inp = raw_input("<ENTER> to re-start, 'Q' to quit \n")
    if (inp == "q" or inp == "Q"):
      break
    main_block.reader.print_results()

  main_block.reader.print_results()
  main_block.stop()
