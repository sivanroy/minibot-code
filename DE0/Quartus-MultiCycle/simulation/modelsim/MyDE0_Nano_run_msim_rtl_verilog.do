transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle {D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle/MySPI.sv}
vlog -sv -work work +incdir+D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle {D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle/MyARM_MultiCycle.sv}
vlog -sv -work work +incdir+D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle {D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle/MyDE0_Nano.sv}

vlog -sv -work work +incdir+D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle {D:/UCL/Q7_LELEC2531/HW7/Quartus-MultiCycle/Quartus-MultiCycle/MyTestbench.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cycloneive_ver -L rtl_work -L work -voptargs="+acc"  MyTestbench

add wave *
view structure
view signals
run -all
