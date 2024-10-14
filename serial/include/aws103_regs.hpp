#pragma once

#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <antenna.hpp>

class AWS103RawRegs
{
public:

    AWS103RawRegs(const std::array<uint16_t, SAS::c_cc_regs_per_cc>& arr);

    uint16_t AWS103_CONTROL;
    uint16_t TX_TVGA_DC_CTRL_CAL;
    uint16_t EL4_RX_BEAM_V_AMP_PH;
    uint16_t EL4_RX_BEAM_H_AMP_PH;
    uint16_t EL4_TX_BEAM_H_AMP_PH;
    uint16_t EL3_RX_BEAM_V_AMP_PH;
    uint16_t EL3_RX_BEAM_H_AMP_PH;
    uint16_t EL3_TX_BEAM_H_AMP_PH;
    uint16_t EL2_RX_BEAM_V_AMP_PH;
    uint16_t EL2_RX_BEAM_H_AMP_PH;
    uint16_t EL2_TX_BEAM_H_AMP_PH;
    uint16_t EL1_RX_BEAM_V_AMP_PH;
    uint16_t EL1_RX_BEAM_H_AMP_PH;
    uint16_t EL1_TX_BEAM_H_AMP_PH;
    uint16_t RXV_RXH_TEMP_VGA;
    uint16_t TX_POUT_EL_3_4;
    uint16_t TX_POUT_EL_1_2;
    uint16_t TEMP_DATA;

    std::string
    to_string();
};

class AWS103Regs
{
public:

    AWS103Regs(const AWS103RawRegs& raw_regs);

    std::string
    to_string();

private:

    double
    convert2celsius(int code);

    int
    tx_tvga2dB(int code);

    int
    rx_tvga2dB(int code);

    int temp;  // Decoded temperature
    int tx2_pout, tx1_pout, tx3_pout, tx4_pout;  // TX power states
    int rxh_tvga, rxv_tvga;  // RX TVGA states
    int TX1_PS, TX1_VGA, RXH1_PS, RXH1_VGA, RXV1_PS, RXV1_VGA;
    int TX2_PS, TX2_VGA, RXH2_PS, RXH2_VGA, RXV2_PS, RXV2_VGA;
    int TX3_PS, TX3_VGA, RXH3_PS, RXH3_VGA, RXV3_PS, RXV3_VGA;
    int TX4_PS, TX4_VGA, RXH4_PS, RXH4_VGA, RXV4_PS, RXV4_VGA;
    int NW, NE, SW, SE;  // Quadrants
    int cal, tx_tvga;  // Calibration and TX TVGA settings
    int tx_en_delay, spi_LDB, FSB, spi_mode;  // Control register settings
};
