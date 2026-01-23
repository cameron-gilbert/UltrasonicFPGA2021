# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\Users\Cam\AX7010_Work\UltrasonicFPGA2021\delay-and-sum-v2\delay-and-sum-v2-app_system\_ide\scripts\systemdebugger_delay-and-sum-v2-app_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\Users\Cam\AX7010_Work\UltrasonicFPGA2021\delay-and-sum-v2\delay-and-sum-v2-app_system\_ide\scripts\systemdebugger_delay-and-sum-v2-app_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent JTAG-HS1 210512180081" && level==0 && jtag_device_ctx=="jsn-JTAG-HS1-210512180081-13722093-0"}
fpga -file C:/Users/Cam/AX7010_Work/UltrasonicFPGA2021/delay-and-sum-v2/delay-and-sum-v2-app/_ide/bitstream/Demodulator_128Ch_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw C:/Users/Cam/AX7010_Work/UltrasonicFPGA2021/delay-and-sum-v2/delay-and-sum-v2-platform/export/delay-and-sum-v2-platform/hw/Demodulator_128Ch_wrapper22.xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
source C:/Users/Cam/AX7010_Work/UltrasonicFPGA2021/delay-and-sum-v2/delay-and-sum-v2-app/_ide/psinit/ps7_init.tcl
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
dow C:/Users/Cam/AX7010_Work/UltrasonicFPGA2021/delay-and-sum-v2/delay-and-sum-v2-app/Debug/delay-and-sum-v2-app.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A9*#0"}
con
