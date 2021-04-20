onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -radix hexadecimal /tbcore/Memory
add wave -noupdate /tbcore/clk
add wave -noupdate /tbcore/reset
add wave -noupdate -radix hexadecimal /tbcore/instAddress
add wave -noupdate /tbcore/instRead
add wave -noupdate -radix hexadecimal /tbcore/instReadData
add wave -noupdate -radix hexadecimal /tbcore/dataAddress
add wave -noupdate /tbcore/dataByteEnable
add wave -noupdate /tbcore/dataWrite
add wave -noupdate -radix hexadecimal /tbcore/dataWriteData
add wave -noupdate /tbcore/dataRead
add wave -noupdate -radix hexadecimal /tbcore/dataReadData
add wave -noupdate /tbcore/UUT/csi_clk
add wave -noupdate /tbcore/UUT/rsi_reset_n
add wave -noupdate -radix hexadecimal /tbcore/UUT/avm_i_address
add wave -noupdate /tbcore/UUT/avm_i_read
add wave -noupdate -radix hexadecimal /tbcore/UUT/avm_i_readdata
add wave -noupdate -radix hexadecimal /tbcore/UUT/i_readdata
add wave -noupdate -radix hexadecimal /tbcore/UUT/avm_d_address
add wave -noupdate /tbcore/UUT/avm_d_write
add wave -noupdate -radix hexadecimal /tbcore/UUT/avm_d_writedata
add wave -noupdate -radix hexadecimal /tbcore/UUT/d_writedata
add wave -noupdate /tbcore/dataByteEnable
add wave -noupdate /tbcore/UUT/avm_d_read
add wave -noupdate -radix hexadecimal /tbcore/UUT/avm_d_readdata
add wave -noupdate -radix hexadecimal /tbcore/UUT/d_readdata
add wave -noupdate -radix hexadecimal -childformat {{/tbcore/UUT/R.curInst -radix hexadecimal} {/tbcore/UUT/R.ctrlState -radix hexadecimal} {/tbcore/UUT/R.curPC -radix hexadecimal} {/tbcore/UUT/R.aluRes -radix hexadecimal} {/tbcore/UUT/R.divStart -radix hexadecimal} {/tbcore/UUT/R.divisor -radix hexadecimal} {/tbcore/UUT/R.dividend -radix hexadecimal} {/tbcore/UUT/R.csrReg -radix hexadecimal -childformat {{/tbcore/UUT/R.csrReg(35) -radix hexadecimal} {/tbcore/UUT/R.csrReg(34) -radix hexadecimal} {/tbcore/UUT/R.csrReg(33) -radix hexadecimal} {/tbcore/UUT/R.csrReg(32) -radix hexadecimal} {/tbcore/UUT/R.csrReg(31) -radix hexadecimal} {/tbcore/UUT/R.csrReg(30) -radix hexadecimal} {/tbcore/UUT/R.csrReg(29) -radix hexadecimal} {/tbcore/UUT/R.csrReg(28) -radix hexadecimal} {/tbcore/UUT/R.csrReg(27) -radix hexadecimal} {/tbcore/UUT/R.csrReg(26) -radix hexadecimal} {/tbcore/UUT/R.csrReg(25) -radix hexadecimal} {/tbcore/UUT/R.csrReg(24) -radix hexadecimal} {/tbcore/UUT/R.csrReg(23) -radix hexadecimal} {/tbcore/UUT/R.csrReg(22) -radix hexadecimal} {/tbcore/UUT/R.csrReg(21) -radix hexadecimal} {/tbcore/UUT/R.csrReg(20) -radix hexadecimal} {/tbcore/UUT/R.csrReg(19) -radix hexadecimal} {/tbcore/UUT/R.csrReg(18) -radix hexadecimal} {/tbcore/UUT/R.csrReg(17) -radix hexadecimal} {/tbcore/UUT/R.csrReg(16) -radix hexadecimal} {/tbcore/UUT/R.csrReg(15) -radix hexadecimal} {/tbcore/UUT/R.csrReg(14) -radix hexadecimal} {/tbcore/UUT/R.csrReg(13) -radix hexadecimal} {/tbcore/UUT/R.csrReg(12) -radix hexadecimal} {/tbcore/UUT/R.csrReg(11) -radix hexadecimal} {/tbcore/UUT/R.csrReg(10) -radix hexadecimal} {/tbcore/UUT/R.csrReg(9) -radix hexadecimal} {/tbcore/UUT/R.csrReg(8) -radix hexadecimal} {/tbcore/UUT/R.csrReg(7) -radix hexadecimal} {/tbcore/UUT/R.csrReg(6) -radix hexadecimal} {/tbcore/UUT/R.csrReg(5) -radix hexadecimal} {/tbcore/UUT/R.csrReg(4) -radix hexadecimal} {/tbcore/UUT/R.csrReg(3) -radix hexadecimal} {/tbcore/UUT/R.csrReg(2) -radix hexadecimal} {/tbcore/UUT/R.csrReg(1) -radix hexadecimal} {/tbcore/UUT/R.csrReg(0) -radix hexadecimal}}}} -expand -subitemconfig {/tbcore/UUT/R.curInst {-height 16 -radix hexadecimal} /tbcore/UUT/R.ctrlState {-height 16 -radix hexadecimal} /tbcore/UUT/R.curPC {-height 16 -radix hexadecimal} /tbcore/UUT/R.aluRes {-height 16 -radix hexadecimal} /tbcore/UUT/R.divStart {-height 15 -radix hexadecimal} /tbcore/UUT/R.divisor {-height 15 -radix hexadecimal} /tbcore/UUT/R.dividend {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg {-height 15 -radix hexadecimal -childformat {{/tbcore/UUT/R.csrReg(35) -radix hexadecimal} {/tbcore/UUT/R.csrReg(34) -radix hexadecimal} {/tbcore/UUT/R.csrReg(33) -radix hexadecimal} {/tbcore/UUT/R.csrReg(32) -radix hexadecimal} {/tbcore/UUT/R.csrReg(31) -radix hexadecimal} {/tbcore/UUT/R.csrReg(30) -radix hexadecimal} {/tbcore/UUT/R.csrReg(29) -radix hexadecimal} {/tbcore/UUT/R.csrReg(28) -radix hexadecimal} {/tbcore/UUT/R.csrReg(27) -radix hexadecimal} {/tbcore/UUT/R.csrReg(26) -radix hexadecimal} {/tbcore/UUT/R.csrReg(25) -radix hexadecimal} {/tbcore/UUT/R.csrReg(24) -radix hexadecimal} {/tbcore/UUT/R.csrReg(23) -radix hexadecimal} {/tbcore/UUT/R.csrReg(22) -radix hexadecimal} {/tbcore/UUT/R.csrReg(21) -radix hexadecimal} {/tbcore/UUT/R.csrReg(20) -radix hexadecimal} {/tbcore/UUT/R.csrReg(19) -radix hexadecimal} {/tbcore/UUT/R.csrReg(18) -radix hexadecimal} {/tbcore/UUT/R.csrReg(17) -radix hexadecimal} {/tbcore/UUT/R.csrReg(16) -radix hexadecimal} {/tbcore/UUT/R.csrReg(15) -radix hexadecimal} {/tbcore/UUT/R.csrReg(14) -radix hexadecimal} {/tbcore/UUT/R.csrReg(13) -radix hexadecimal} {/tbcore/UUT/R.csrReg(12) -radix hexadecimal} {/tbcore/UUT/R.csrReg(11) -radix hexadecimal} {/tbcore/UUT/R.csrReg(10) -radix hexadecimal} {/tbcore/UUT/R.csrReg(9) -radix hexadecimal} {/tbcore/UUT/R.csrReg(8) -radix hexadecimal} {/tbcore/UUT/R.csrReg(7) -radix hexadecimal} {/tbcore/UUT/R.csrReg(6) -radix hexadecimal} {/tbcore/UUT/R.csrReg(5) -radix hexadecimal} {/tbcore/UUT/R.csrReg(4) -radix hexadecimal} {/tbcore/UUT/R.csrReg(3) -radix hexadecimal} {/tbcore/UUT/R.csrReg(2) -radix hexadecimal} {/tbcore/UUT/R.csrReg(1) -radix hexadecimal} {/tbcore/UUT/R.csrReg(0) -radix hexadecimal}}} /tbcore/UUT/R.csrReg(35) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(34) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(33) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(32) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(31) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(30) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(29) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(28) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(27) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(26) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(25) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(24) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(23) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(22) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(21) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(20) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(19) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(18) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(17) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(16) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(15) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(14) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(13) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(12) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(11) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(10) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(9) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(8) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(7) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(6) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(5) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(4) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(3) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(2) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(1) {-height 15 -radix hexadecimal} /tbcore/UUT/R.csrReg(0) {-height 15 -radix hexadecimal}} /tbcore/UUT/R
add wave -noupdate -radix hexadecimal /tbcore/UUT/NxR
add wave -noupdate -childformat {{/tbcore/UUT/RAM(0) -radix hexadecimal} {/tbcore/UUT/RAM(1) -radix hexadecimal} {/tbcore/UUT/RAM(2) -radix hexadecimal} {/tbcore/UUT/RAM(3) -radix hexadecimal} {/tbcore/UUT/RAM(4) -radix hexadecimal} {/tbcore/UUT/RAM(5) -radix hexadecimal} {/tbcore/UUT/RAM(6) -radix hexadecimal} {/tbcore/UUT/RAM(7) -radix hexadecimal} {/tbcore/UUT/RAM(8) -radix hexadecimal} {/tbcore/UUT/RAM(9) -radix hexadecimal} {/tbcore/UUT/RAM(10) -radix hexadecimal} {/tbcore/UUT/RAM(11) -radix hexadecimal} {/tbcore/UUT/RAM(12) -radix hexadecimal} {/tbcore/UUT/RAM(13) -radix hexadecimal} {/tbcore/UUT/RAM(14) -radix hexadecimal} {/tbcore/UUT/RAM(15) -radix hexadecimal} {/tbcore/UUT/RAM(16) -radix hexadecimal} {/tbcore/UUT/RAM(17) -radix hexadecimal} {/tbcore/UUT/RAM(18) -radix hexadecimal} {/tbcore/UUT/RAM(19) -radix hexadecimal} {/tbcore/UUT/RAM(20) -radix hexadecimal} {/tbcore/UUT/RAM(21) -radix hexadecimal} {/tbcore/UUT/RAM(22) -radix hexadecimal} {/tbcore/UUT/RAM(23) -radix hexadecimal} {/tbcore/UUT/RAM(24) -radix hexadecimal} {/tbcore/UUT/RAM(25) -radix hexadecimal} {/tbcore/UUT/RAM(26) -radix hexadecimal} {/tbcore/UUT/RAM(27) -radix hexadecimal} {/tbcore/UUT/RAM(28) -radix hexadecimal} {/tbcore/UUT/RAM(29) -radix hexadecimal} {/tbcore/UUT/RAM(30) -radix hexadecimal} {/tbcore/UUT/RAM(31) -radix hexadecimal} {/tbcore/UUT/RAM(32) -radix hexadecimal} {/tbcore/UUT/RAM(33) -radix hexadecimal} {/tbcore/UUT/RAM(34) -radix hexadecimal} {/tbcore/UUT/RAM(35) -radix hexadecimal} {/tbcore/UUT/RAM(36) -radix hexadecimal} {/tbcore/UUT/RAM(37) -radix hexadecimal} {/tbcore/UUT/RAM(38) -radix hexadecimal} {/tbcore/UUT/RAM(39) -radix hexadecimal} {/tbcore/UUT/RAM(40) -radix hexadecimal} {/tbcore/UUT/RAM(41) -radix hexadecimal} {/tbcore/UUT/RAM(42) -radix hexadecimal} {/tbcore/UUT/RAM(43) -radix hexadecimal} {/tbcore/UUT/RAM(44) -radix hexadecimal} {/tbcore/UUT/RAM(45) -radix hexadecimal} {/tbcore/UUT/RAM(46) -radix hexadecimal} {/tbcore/UUT/RAM(47) -radix hexadecimal} {/tbcore/UUT/RAM(48) -radix hexadecimal} {/tbcore/UUT/RAM(49) -radix hexadecimal} {/tbcore/UUT/RAM(50) -radix hexadecimal} {/tbcore/UUT/RAM(51) -radix hexadecimal} {/tbcore/UUT/RAM(52) -radix hexadecimal} {/tbcore/UUT/RAM(53) -radix hexadecimal} {/tbcore/UUT/RAM(54) -radix hexadecimal} {/tbcore/UUT/RAM(55) -radix hexadecimal} {/tbcore/UUT/RAM(56) -radix hexadecimal} {/tbcore/UUT/RAM(57) -radix hexadecimal} {/tbcore/UUT/RAM(58) -radix hexadecimal} {/tbcore/UUT/RAM(59) -radix hexadecimal} {/tbcore/UUT/RAM(60) -radix hexadecimal} {/tbcore/UUT/RAM(61) -radix hexadecimal} {/tbcore/UUT/RAM(62) -radix hexadecimal} {/tbcore/UUT/RAM(63) -radix hexadecimal} {/tbcore/UUT/RAM(64) -radix hexadecimal} {/tbcore/UUT/RAM(65) -radix hexadecimal} {/tbcore/UUT/RAM(66) -radix hexadecimal} {/tbcore/UUT/RAM(67) -radix hexadecimal}} -subitemconfig {/tbcore/UUT/RAM(0) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(1) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(2) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(3) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(4) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(5) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(6) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(7) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(8) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(9) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(10) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(11) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(12) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(13) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(14) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(15) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(16) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(17) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(18) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(19) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(20) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(21) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(22) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(23) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(24) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(25) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(26) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(27) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(28) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(29) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(30) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(31) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(32) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(33) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(34) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(35) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(36) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(37) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(38) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(39) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(40) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(41) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(42) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(43) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(44) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(45) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(46) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(47) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(48) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(49) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(50) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(51) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(52) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(53) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(54) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(55) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(56) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(57) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(58) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(59) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(60) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(61) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(62) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(63) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(64) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(65) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(66) {-height 15 -radix hexadecimal} /tbcore/UUT/RAM(67) {-height 15 -radix hexadecimal}} /tbcore/UUT/RAM
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {12078219 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 264
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {11522308 ps} {13340932 ps}
