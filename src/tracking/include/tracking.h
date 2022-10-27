/******************************************************************************
 *  @file       tracking.h
 *
 *  <!-- Start of section for manual description -->
 *  @brief      get the position of courier
 *  <!-- End of section for manual description -->
 *
 *  @author     Stefan Sass <stefan.sass@ovgu.de>
 *  @author

 ******************************************************************************/
#pragma once


namespace Tracking {

    // constants
    const bool PUBLISH_TF = true;
    const std::string REF_FRAME = "map";
    const std::string FRAME = "tracker";
    const std::string PORTNAME = "/dev/ttyUSB0";
    const int SERIAL_TIMEOUT = 800;
    const int TRACKER_STREAM_SHORT_LEN = 4;
    constexpr double MM_TO_M_FACTOR = 0.001;


/// function for split string
/// @brief split string by given delimiter
/// @param string input_line
/// @param char delimiter
/// @return std::vector<std::string>
    std::vector <std::string> splitString(const std::string &input_line, char delimiter) {
        std::vector <std::string> fields;
        std::string field;
        std::istringstream line(input_line);

        while (std::getline(line, field, delimiter)) {
            fields.push_back(field);
        }
        return fields;
    }

}

