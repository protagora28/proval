#include <aws103_regs.hpp>

AWS103RawRegs::AWS103RawRegs(const std::array<uint16_t, SAS::c_cc_regs_per_cc>& arr)
{
    AWS103_CONTROL = arr[17];
    TX_TVGA_DC_CTRL_CAL = arr[16];
    EL4_RX_BEAM_V_AMP_PH = arr[15];
    EL4_RX_BEAM_H_AMP_PH = arr[14];
    EL4_TX_BEAM_H_AMP_PH = arr[13];
    EL3_RX_BEAM_V_AMP_PH = arr[12];
    EL3_RX_BEAM_H_AMP_PH = arr[11];
    EL3_TX_BEAM_H_AMP_PH = arr[10];
    EL2_RX_BEAM_V_AMP_PH = arr[9];
    EL2_RX_BEAM_H_AMP_PH = arr[8];
    EL2_TX_BEAM_H_AMP_PH = arr[7];
    EL1_RX_BEAM_V_AMP_PH = arr[6];
    EL1_RX_BEAM_H_AMP_PH = arr[5];
    EL1_TX_BEAM_H_AMP_PH = arr[4];
    RXV_RXH_TEMP_VGA = arr[3];
    TX_POUT_EL_3_4 = arr[2];
    TX_POUT_EL_1_2 = arr[1];
    TEMP_DATA = arr[0];
}

std::string
AWS103RawRegs::to_string()
{
    std::stringstream rp;

    rp << "TEMP_DATA: " << std::hex << std::setw(4) << TEMP_DATA << "\n";
    rp << "TX_POUT_EL_1_2: " << std::hex << std::setw(4) << TX_POUT_EL_1_2 << "\n";
    rp << "TX_POUT_EL_3_4: " << std::hex << std::setw(4) << TX_POUT_EL_3_4 << "\n";
    rp << "RXV_RXH_TEMP_VGA: " << std::hex << std::setw(4) << RXV_RXH_TEMP_VGA << "\n";
    rp << "EL1_TX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL1_TX_BEAM_H_AMP_PH << "\n";
    rp << "EL1_RX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL1_RX_BEAM_H_AMP_PH << "\n";
    rp << "EL1_RX_BEAM_V_AMP_PH: " << std::hex << std::setw(4) << EL1_RX_BEAM_V_AMP_PH << "\n";
    rp << "EL2_TX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL2_TX_BEAM_H_AMP_PH << "\n";
    rp << "EL2_RX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL2_RX_BEAM_H_AMP_PH << "\n";
    rp << "EL2_RX_BEAM_V_AMP_PH: " << std::hex << std::setw(4) << EL2_RX_BEAM_V_AMP_PH << "\n";
    rp << "EL3_TX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL3_TX_BEAM_H_AMP_PH << "\n";
    rp << "EL3_RX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL3_RX_BEAM_H_AMP_PH << "\n";
    rp << "EL3_RX_BEAM_V_AMP_PH: " << std::hex << std::setw(4) << EL3_RX_BEAM_V_AMP_PH << "\n";
    rp << "EL4_TX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL4_TX_BEAM_H_AMP_PH << "\n";
    rp << "EL4_RX_BEAM_H_AMP_PH: " << std::hex << std::setw(4) << EL4_RX_BEAM_H_AMP_PH << "\n";
    rp << "EL4_RX_BEAM_V_AMP_PH: " << std::hex << std::setw(4) << EL4_RX_BEAM_V_AMP_PH << "\n";
    rp << "TX_TVGA_DC_CTRL_CAL: " << std::hex << std::setw(4) << TX_TVGA_DC_CTRL_CAL << "\n";
    rp << "AWS103_CONTROL: " << std::hex << std::setw(4) << AWS103_CONTROL << "\n";

    return rp.str();
}

AWS103Regs::AWS103Regs(const AWS103RawRegs& raw_regs)
{
    const auto get_control_register_bits = [](uint16_t reg, int pos, int mask) -> int
    {
        return (mask & (int)reg) >> pos;
    };

    // AWS103_CONTROL.
    constexpr int TX_EN_DELAY_Pos = 1;
    constexpr int TX_EN_DELAY_Msk = (0x7 << TX_EN_DELAY_Pos);
    constexpr int SPI_LDB_DELAY_Pos = 4;
    constexpr int SPI_LDB_DELAY_Msk = (0x7 << SPI_LDB_DELAY_Pos);
    constexpr int FBS_WEIGHT_ADDRESS_Pos = 7;
    constexpr int FBS_WEIGHT_ADDRESS_Msk = (0x7 << FBS_WEIGHT_ADDRESS_Pos);
    constexpr int SPI_LOAD_MODE_Pos = 10;
    constexpr int SPI_LOAD_MODE_Msk = (0x3 << SPI_LOAD_MODE_Pos);

    tx_en_delay = get_control_register_bits(raw_regs.AWS103_CONTROL, TX_EN_DELAY_Pos, TX_EN_DELAY_Msk);
    spi_LDB = get_control_register_bits(raw_regs.AWS103_CONTROL, SPI_LDB_DELAY_Pos, SPI_LDB_DELAY_Msk);
    FSB = get_control_register_bits(raw_regs.AWS103_CONTROL, FBS_WEIGHT_ADDRESS_Pos, FBS_WEIGHT_ADDRESS_Msk);
    spi_mode = get_control_register_bits(raw_regs.AWS103_CONTROL, SPI_LOAD_MODE_Pos, SPI_LOAD_MODE_Msk);

    // TX_TVGA_DC_CTRL_CAL.
    constexpr int NW_QUADRANT_Pos = 0;
    constexpr int NW_QUADRANT_Msk = (0x1 << NW_QUADRANT_Pos);
    constexpr int NE_QUADRANT_Pos = 1;
    constexpr int NE_QUADRANT_Msk = (0x1 << NE_QUADRANT_Pos);
    constexpr int SW_QUADRANT_Pos = 2;
    constexpr int SW_QUADRANT_Msk = (0x1 << SW_QUADRANT_Pos);
    constexpr int SE_QUADRANT_Pos = 3;
    constexpr int SE_QUADRANT_Msk = (0x1 << SE_QUADRANT_Pos);
    constexpr int CAL_Pos = 4;
    constexpr int CAL_Msk = (0x7 << CAL_Pos);
    constexpr int TX_TVGA_Pos = 7;
    constexpr int TX_TVGA_Msk = (0x1F << TX_TVGA_Pos);

    NW = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, NW_QUADRANT_Pos, NW_QUADRANT_Msk);
    NE = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, NE_QUADRANT_Pos, NE_QUADRANT_Msk);
    SW = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, SW_QUADRANT_Pos, SW_QUADRANT_Msk);
    SE = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, SE_QUADRANT_Pos, SE_QUADRANT_Msk);
    cal = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, CAL_Pos, CAL_Msk);
    tx_tvga = get_control_register_bits(raw_regs.TX_TVGA_DC_CTRL_CAL, TX_TVGA_Pos, TX_TVGA_Msk);

    // EL4_RX_BEAM_V_AMP_PH.
    constexpr int RX4_PS_STATE_Pos = 0;
    constexpr int RX4_PS_STATE_Msk = (0x3F << RX4_PS_STATE_Pos);
    constexpr int RX4_VGA_STATE_Pos = 6;
    constexpr int RX4_VGA_STATE_Msk = (0x3F << RX4_VGA_STATE_Pos);

    RXV4_PS = get_control_register_bits(raw_regs.EL4_RX_BEAM_V_AMP_PH, RX4_PS_STATE_Pos, RX4_PS_STATE_Msk);
    RXV4_VGA = get_control_register_bits(raw_regs.EL4_RX_BEAM_V_AMP_PH, RX4_VGA_STATE_Pos, RX4_VGA_STATE_Msk);

    // EL4_RX_BEAM_H_AMP_PH.
    constexpr int RXH4_PS_STATE_Pos = 0;
    constexpr int RXH4_PS_STATE_Msk = (0x3F << RXH4_PS_STATE_Pos);
    constexpr int RXH4_VGA_STATE_Pos = 6;
    constexpr int RXH4_VGA_STATE_Msk = (0x3F << RXH4_VGA_STATE_Pos);

    RXH4_PS = get_control_register_bits(raw_regs.EL4_RX_BEAM_H_AMP_PH, RXH4_PS_STATE_Pos, RXH4_PS_STATE_Msk);
    RXH4_VGA = get_control_register_bits(raw_regs.EL4_RX_BEAM_H_AMP_PH, RXH4_VGA_STATE_Pos, RXH4_VGA_STATE_Msk);

    // EL4_TX_BEAM_H_AMP_PH.
    constexpr int TXH4_PS_STATE_Pos = 0;
    constexpr int TXH4_PS_STATE_Msk = (0x3F << TXH4_PS_STATE_Pos);
    constexpr int TXH4_VGA_STATE_Pos = 6;
    constexpr int TXH4_VGA_STATE_Msk = (0x3F << TXH4_VGA_STATE_Pos);

    TX4_PS = get_control_register_bits(raw_regs.EL4_TX_BEAM_H_AMP_PH, TXH4_PS_STATE_Pos, TXH4_PS_STATE_Msk);
    TX4_VGA = get_control_register_bits(raw_regs.EL4_TX_BEAM_H_AMP_PH, TXH4_VGA_STATE_Pos, TXH4_VGA_STATE_Msk);

    // EL3_RX_BEAM_V_AMP_PH.
    constexpr int RX3_PS_STATE_Pos = 0;
    constexpr int RX3_PS_STATE_Msk = (0x3F << RX3_PS_STATE_Pos);
    constexpr int RX3_VGA_STATE_Pos = 6;
    constexpr int RX3_VGA_STATE_Msk = (0x3F << RX3_VGA_STATE_Pos);

    RXV3_PS = get_control_register_bits(raw_regs.EL3_RX_BEAM_V_AMP_PH, RX3_PS_STATE_Pos, RX3_PS_STATE_Msk);
    RXV3_VGA = get_control_register_bits(raw_regs.EL3_RX_BEAM_V_AMP_PH, RX3_VGA_STATE_Pos, RX3_VGA_STATE_Msk);

    // EL3_RX_BEAM_H_AMP_PH.
    constexpr int RXH3_PS_STATE_Pos = 0;
    constexpr int RXH3_PS_STATE_Msk = (0x3F << RXH3_PS_STATE_Pos);
    constexpr int RXH3_VGA_STATE_Pos = 6;
    constexpr int RXH3_VGA_STATE_Msk = (0x3F << RXH3_VGA_STATE_Pos);

    RXH3_PS = get_control_register_bits(raw_regs.EL3_RX_BEAM_H_AMP_PH, RXH3_PS_STATE_Pos, RXH3_PS_STATE_Msk);
    RXH3_VGA = get_control_register_bits(raw_regs.EL3_RX_BEAM_H_AMP_PH, RXH3_VGA_STATE_Pos, RXH3_VGA_STATE_Msk);

    // EL3_TX_BEAM_H_AMP_PH.
    constexpr int TXH3_PS_STATE_Pos = 0;
    constexpr int TXH3_PS_STATE_Msk = (0x3F << TXH3_PS_STATE_Pos);
    constexpr int TXH3_VGA_STATE_Pos = 6;
    constexpr int TXH3_VGA_STATE_Msk = (0x3F << TXH3_VGA_STATE_Pos);

    TX3_PS = get_control_register_bits(raw_regs.EL3_TX_BEAM_H_AMP_PH, TXH3_PS_STATE_Pos, TXH3_PS_STATE_Msk);
    TX3_VGA = get_control_register_bits(raw_regs.EL3_TX_BEAM_H_AMP_PH, TXH3_VGA_STATE_Pos, TXH3_VGA_STATE_Msk);

    // EL2_RX_BEAM_V_AMP_PH.
    constexpr int RX2_PS_STATE_Pos = 0;
    constexpr int RX2_PS_STATE_Msk = (0x3F << RX2_PS_STATE_Pos);
    constexpr int RX2_VGA_STATE_Pos = 6;
    constexpr int RX2_VGA_STATE_Msk = (0x3F << RX2_VGA_STATE_Pos);

    RXV2_PS = get_control_register_bits(raw_regs.EL2_RX_BEAM_V_AMP_PH, RX2_PS_STATE_Pos, RX2_PS_STATE_Msk);
    RXV2_VGA = get_control_register_bits(raw_regs.EL2_RX_BEAM_V_AMP_PH, RX2_VGA_STATE_Pos, RX2_VGA_STATE_Msk);

    // EL2_RX_BEAM_H_AMP_PH.
    constexpr int RXH2_PS_STATE_Pos = 0;
    constexpr int RXH2_PS_STATE_Msk = (0x3F << RXH2_PS_STATE_Pos);
    constexpr int RXH2_VGA_STATE_Pos = 6;
    constexpr int RXH2_VGA_STATE_Msk = (0x3F << RXH2_VGA_STATE_Pos);

    RXH2_PS = get_control_register_bits(raw_regs.EL2_RX_BEAM_H_AMP_PH, RXH2_PS_STATE_Pos, RXH2_PS_STATE_Msk);
    RXH2_VGA = get_control_register_bits(raw_regs.EL2_RX_BEAM_H_AMP_PH, RXH2_VGA_STATE_Pos, RXH2_VGA_STATE_Msk);

    // EL2_TX_BEAM_H_AMP_PH.
    constexpr int TXH2_PS_STATE_Pos = 0;
    constexpr int TXH2_PS_STATE_Msk = (0x3F << TXH2_PS_STATE_Pos);
    constexpr int TXH2_VGA_STATE_Pos = 6;
    constexpr int TXH2_VGA_STATE_Msk = (0x3F << TXH2_VGA_STATE_Pos);

    TX2_PS = get_control_register_bits(raw_regs.EL2_TX_BEAM_H_AMP_PH, TXH2_PS_STATE_Pos, TXH2_PS_STATE_Msk);
    TX2_VGA = get_control_register_bits(raw_regs.EL2_TX_BEAM_H_AMP_PH, TXH2_VGA_STATE_Pos, TXH2_VGA_STATE_Msk);

    // EL1_RX_BEAM_V_AMP_PH.
    constexpr int RX1_PS_STATE_Pos = 0;
    constexpr int RX1_PS_STATE_Msk = (0x3F << RX1_PS_STATE_Pos);
    constexpr int RX1_VGA_STATE_Pos = 6;
    constexpr int RX1_VGA_STATE_Msk = (0x3F << RX1_VGA_STATE_Pos);

    RXV1_PS = get_control_register_bits(raw_regs.EL1_RX_BEAM_V_AMP_PH, RX1_PS_STATE_Pos, RX1_PS_STATE_Msk);
    RXV1_VGA = get_control_register_bits(raw_regs.EL1_RX_BEAM_V_AMP_PH, RX1_VGA_STATE_Pos, RX1_VGA_STATE_Msk);

    // EL1_RX_BEAM_H_AMP_PH.
    constexpr int RXH1_PS_STATE_Pos = 0;
    constexpr int RXH1_PS_STATE_Msk = (0x3F << RXH1_PS_STATE_Pos);
    constexpr int RXH1_VGA_STATE_Pos = 6;
    constexpr int RXH1_VGA_STATE_Msk = (0x3F << RXH1_VGA_STATE_Pos);

    RXH1_PS = get_control_register_bits(raw_regs.EL1_RX_BEAM_H_AMP_PH, RXH1_PS_STATE_Pos, RXH1_PS_STATE_Msk);
    RXH1_VGA = get_control_register_bits(raw_regs.EL1_RX_BEAM_H_AMP_PH, RXH1_VGA_STATE_Pos, RXH1_VGA_STATE_Msk);

    // EL1_TX_BEAM_H_AMP_PH.
    constexpr int TXH1_PS_STATE_Pos = 0;
    constexpr int TXH1_PS_STATE_Msk = (0x3F << TXH1_PS_STATE_Pos);
    constexpr int TXH1_VGA_STATE_Pos = 6;
    constexpr int TXH1_VGA_STATE_Msk = (0x3F << TXH1_VGA_STATE_Pos);

    TX1_PS = get_control_register_bits(raw_regs.EL1_TX_BEAM_H_AMP_PH, TXH1_PS_STATE_Pos, TXH1_PS_STATE_Msk);
    TX1_VGA = get_control_register_bits(raw_regs.EL1_TX_BEAM_H_AMP_PH, TXH1_VGA_STATE_Pos, TXH1_VGA_STATE_Msk);

    // RXV_RXH_TEMP_VGA.
    constexpr int RXH_TVGA_STATE_Pos = 0;
    constexpr int RXH_TVGA_STATE_Msk = (0x3F << RXH_TVGA_STATE_Pos);
    constexpr int RXV_TVGA_STATE_Pos = 6;
    constexpr int RXV_TVGA_STATE_Msk = (0x3F << RXV_TVGA_STATE_Pos);

    rxv_tvga = get_control_register_bits(raw_regs.RXV_RXH_TEMP_VGA, RXV_TVGA_STATE_Pos, RXV_TVGA_STATE_Msk);
    rxh_tvga = get_control_register_bits(raw_regs.RXV_RXH_TEMP_VGA, RXH_TVGA_STATE_Pos, RXH_TVGA_STATE_Msk);

    // TX_POUT_EL_3_4.
    constexpr int TX3_POUT_Pos = 0;
    constexpr int TX3_POUT_STATE_Msk = (0x1F << TX3_POUT_Pos);
    constexpr int TX4_POUT_STATE_Pos = 6;
    constexpr int TX4_POUT_STATE_Msk = (0x1F << TX4_POUT_STATE_Pos);

    tx3_pout = get_control_register_bits(raw_regs.TX_POUT_EL_3_4, TX3_POUT_Pos, TX3_POUT_STATE_Msk);
    tx4_pout = get_control_register_bits(raw_regs.TX_POUT_EL_3_4, TX4_POUT_STATE_Pos, TX4_POUT_STATE_Msk);

    // TX_POUT_EL_1_2.
    constexpr int TX2_POUT_Pos = 0;
    constexpr int TX2_POUT_STATE_Msk = (0x1F << TX2_POUT_Pos);
    constexpr int TX1_POUT_STATE_Pos = 6;
    constexpr int TX1_POUT_STATE_Msk = (0x1F << TX1_POUT_STATE_Pos);

    tx2_pout = get_control_register_bits(raw_regs.TX_POUT_EL_1_2, TX2_POUT_Pos, TX2_POUT_STATE_Msk);
    tx1_pout = get_control_register_bits(raw_regs.TX_POUT_EL_1_2, TX1_POUT_STATE_Pos, TX1_POUT_STATE_Msk);

    // TEMP_DATA.
    constexpr int TEMP_SENS_Pos = 6;
    constexpr int TEMP_SENS_Msk = (0x1F << TEMP_SENS_Pos);

    temp = get_control_register_bits(raw_regs.TEMP_DATA, TEMP_SENS_Pos, TEMP_SENS_Msk);
}

std::string
AWS103Regs::to_string()
{
    std::stringstream rp;

    rp << "Temp: " << std::to_string(convert2celsius(temp)) << " " << std::to_string(temp) << "\n";

    rp << "TX_tvga: " << std::to_string(tx_tvga2dB(tx_tvga)) << " dB  ";
    rp << "RXV_tvga: " << std::to_string(rx_tvga2dB(rxv_tvga)) << " dB  ";
    rp << "RXH_tvga: " << std::to_string(rx_tvga2dB(rxh_tvga)) << " dB\n";

    rp << "TX PS  [deg]\t";
    rp << std::to_string(TX1_PS * 5.625) << "\t";
    rp << std::to_string(TX2_PS * 5.625) << "\t";
    rp << std::to_string(TX3_PS * 5.625) << "\t";
    rp << std::to_string(TX4_PS * 5.625) << "\n";

    rp << "TX VGA  [dB]\t";
    rp << std::to_string(TX1_VGA * 0.5) << "\t";
    rp << std::to_string(TX2_VGA * 0.5) << "\t";
    rp << std::to_string(TX3_VGA * 0.5) << "\t";
    rp << std::to_string(TX4_VGA * 0.5) << "\n";

    rp << "RXH PS [deg]\t";
    rp << std::to_string(RXH1_PS * 5.625) << "\t";
    rp << std::to_string(RXH2_PS * 5.625) << "\t";
    rp << std::to_string(RXH3_PS * 5.625) << "\t";
    rp << std::to_string(RXH4_PS * 5.625) << "\n";

    rp << "RXH VGA [dB]\t";
    rp << std::to_string(RXH1_VGA * 0.5) << "\t";
    rp << std::to_string(RXH2_VGA * 0.5) << "\t";
    rp << std::to_string(RXH3_VGA * 0.5) << "\t";
    rp << std::to_string(RXH4_VGA * 0.5) << "\n";

    rp << "RXV PS [deg]\t";
    rp << std::to_string(RXV1_PS * 5.625) << "\t";
    rp << std::to_string(RXV2_PS * 5.625) << "\t";
    rp << std::to_string(RXV3_PS * 5.625) << "\t";
    rp << std::to_string(RXV4_PS * 5.625) << "\n";

    rp << "RXV VGA [dB]\t";
    rp << std::to_string(RXV1_VGA * 0.5) << "\t";
    rp << std::to_string(RXV2_VGA * 0.5) << "\t";
    rp << std::to_string(RXV3_VGA * 0.5) << "\t";
    rp << std::to_string(RXV4_VGA * 0.5) << "\n";

    rp << "NW: " << std::to_string(NW) << "  ";
    rp << "NE: " << std::to_string(NE) << "  ";
    rp << "SW: " << std::to_string(SW) << "  ";
    rp << "SE: " << std::to_string(SE) << "\n";

    rp << "Cal: " << std::to_string(cal) << "  ";
    rp << "TX TVGA: " << std::to_string(tx_tvga) << "\n";

    rp << "TX_EN_DELAY: " << std::to_string(tx_en_delay) << "  ";
    rp << "SPI_LDB: " << std::to_string(spi_LDB) << "  ";
    rp << "FSB: " << std::to_string(FSB) << "  ";
    rp << "SPI_MODE: " << std::to_string(spi_mode) << "\n";

    return rp.str();
}

double
AWS103Regs::convert2celsius(int code)
{
    if (code < 0)
    {
        code = 0;
    }
    if (code > 31)
    {
        code = 31;
    }
    const double celsius = (-65.0 * (double)code + 1300.0) / 10.0;
    return celsius;
}

int
AWS103Regs::tx_tvga2dB(int code)
{
    return (code & 0x1) * 8 + (code >> 1);
}

int
AWS103Regs::rx_tvga2dB(int code)
{
    return (code & 0x1) * 8 + ((code >> 1) & 0x1) * 4 + ((code >> 2) & 0x1) * 8 + ((code >> 3) & 0x1) * 1 +
        ((code >> 4) & 0x1) * 2 + ((code >> 5) & 0x1) * 8;
}
