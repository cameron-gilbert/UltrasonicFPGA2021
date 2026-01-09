# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\CamLT\AX7010_Work\UltrasonicFPGA2021\Vitis\design_1_wrapper\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\CamLT\AX7010_Work\UltrasonicFPGA2021\Vitis\design_1_wrapper\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {design_1_wrapper}\
-hw {C:\Users\CamLT\AX7010_Work\UltrasonicFPGA2021\Vivado\ultrasonic_fpga_2021\design_1_wrapper.xsa}\
-proc {ps7_cortexa9_0} -os {standalone} -out {C:/Users/CamLT/AX7010_Work/UltrasonicFPGA2021/Vitis}

platform write
platform generate -domains 
platform active {design_1_wrapper}
domain active {zynq_fsbl}
bsp reload
domain active {standalone_domain}
bsp reload
bsp setlib -name lwip211 -ver 1.5
bsp removelib -name lwip211
bsp setlib -name lwip211 -ver 1.5
bsp write
bsp reload
catch {bsp regenerate}
platform generate
platform clean
platform generate
platform clean
platform generate
platform generate -domains standalone_domain,zynq_fsbl 
