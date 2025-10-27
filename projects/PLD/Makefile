%.jed: %.pld
	@echo -n Compiling $< into $@...
	@galette $< 
	@echo done.

all: NewCPUCardDecoder.jed \
	RPiCH351Interface.jed \
	RPiCH351Interface_deactivated.jed \
	SerialEngineDecoderChip.jed \
	ParallelDataCaptureMux.jed \
	ParallelDataCaptureMuxClockChip.jed \
	DenAndADSDetect.jed \
	SerialDataLineInterface.jed \
	DataLine8WriteAccelerator.jed

install_RPiCH351Interface: RPiCH351Interface.jed
	minipro -p "ATF22V10CQZ(UES)" --write $<
install_RPiCH351Interface_deactivated: RPiCH351Interface_deactivated.jed
	minipro -p "ATF22V10CQZ(UES)" --write $<
install_NewCPUCardDecoder_C: NewCPUCardDecoder.jed
	minipro -p ATF16V8C --write $< 
install_NewCPUCardDecoder_B: NewCPUCardDecoder.jed
	minipro -p ATF16V8B --write $< 

install_SerialEngineDecoderChip: SerialEngineDecoderChip.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $< 
install_ParallelDataCaptureMux: ParallelDataCaptureMux.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $< 

install_ParallelDataCaptureMuxClockChip: ParallelDataCaptureMuxClockChip.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $< 

install_DenAndADSDetect: DenAndADSDetect.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $< 

install_SerialDataLineInterface: SerialDataLineInterface.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $<

install_DataLine8WriteAccelerator: DataLine8WriteAccelerator.jed
	minipro -p 'ATF22V10CQZ(UES)' --write $<

clean:
	@echo -n "Cleaning..."
	@rm -rf *.chp *.fus *.jed *.pin
	@echo "done"

.PHONY: help clean


