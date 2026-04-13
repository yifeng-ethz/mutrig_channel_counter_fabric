package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. dashboard_infra cmsis_svd lib mu3e_cmsis_svd.tcl]]
source $helper_file

namespace eval ::mu3e::cmsis::spec {}

proc ::mu3e::cmsis::spec::build_device {} {
    set registers [list \
        [::mu3e::cmsis::svd::register RESET_INTERVAL 0x0 \
            -description "Runtime reset-interval control word for the channel counter fabric." \
            -access read-write \
            -fields [list \
                [::mu3e::cmsis::svd::field value 0 32 \
                    -description "Raw interval configuration word." \
                    -access read-write]]]]

    return [::mu3e::cmsis::svd::device MU3E_AVSTHIT0CH2CNTR_CTRL \
        -version 1.0.0 \
        -description "CMSIS-SVD description of the avsthit0ch2cntr_ctrl avmm_rst_interval aperture." \
        -peripherals [list \
            [::mu3e::cmsis::svd::peripheral AVSTHIT0CH2CNTR_CTRL 0x0 \
                -description "Relative single-word control aperture for the channel-counter reset interval." \
                -groupName MU3E_DATA_PATH \
                -addressBlockSize 0x4 \
                -registers $registers]]]
}

if {[info exists ::argv0] &&
    [file normalize $::argv0] eq [file normalize [info script]]} {
    set out_path [file join $script_dir avsthit0ch2cntr_ctrl.svd]
    if {[llength $::argv] >= 1} {
        set out_path [lindex $::argv 0]
    }
    ::mu3e::cmsis::svd::write_device_file \
        [::mu3e::cmsis::spec::build_device] $out_path
}
