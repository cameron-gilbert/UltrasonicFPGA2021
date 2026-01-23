# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\CamLT\AX7010_Work\UltrasonicFPGA2021\delay-and-sum-v2\delay-and-sum-v2-platform\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\CamLT\AX7010_Work\UltrasonicFPGA2021\delay-and-sum-v2\delay-and-sum-v2-platform\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {delay-and-sum-v2-platform}\
-hw {C:\Users\CamLT\AX7010_Work\Demodulator_128Ch_wrapper.xsa}\
-proc {ps7_cortexa9_0} -os {standalone} -out {C:/Users/CamLT/AX7010_Work/UltrasonicFPGA2021/delay-and-sum-v2}

platform write
platform generate -domains 
platform active {delay-and-sum-v2-platform}
domain active {zynq_fsbl}
bsp reload
bsp setlib -name lwip211 -ver 1.5
bsp write
bsp reload
catch {bsp regenerate}
domain active {standalone_domain}
bsp reload
bsp setlib -name lwip211 -ver 1.5
bsp write
bsp reload
catch {bsp regenerate}
platform generate
platform clean
platform generate
platform config -updatehw {C:/Users/CamLT/AX7010_Work/Demodulator_128Ch_wrapper22.xsa}
platform clean
domain active {zynq_fsbl}
domain active {standalone_domain}
platform generate
bsp reload
bsp reload
bsp write
platform generate -domains 
platform clean
platform generate
platform clean
platform generate
platform generate
