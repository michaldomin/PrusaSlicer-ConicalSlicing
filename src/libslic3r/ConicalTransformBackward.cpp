#include "ConicalTransform.hpp"

namespace Slic3r {

 std::string ConicalTransform::applyBackTransform(const std::string &gcodeLayer, double height) const
 {
     if (_config.opt_float("planar_height") + 0.01 > height) {
         std::smatch g_match;
         std::string last_match;

         auto start = gcodeLayer.cbegin();
         auto end = gcodeLayer.cend();

         while (std::regex_search(start, end, g_match, _patternG)) {
             last_match = g_match.str();
             start = g_match.suffix().first;
         }

         std::smatch x_match, y_match;
         bool x_found = std::regex_search(last_match, x_match, _patternX);
         bool y_found = std::regex_search(last_match, y_match, _patternY);

         if (x_found) {
             _xOld = std::stod(x_match.str().substr(1)) - _centerX;
         }
         if (y_found) {
             _yOld = std::stod(y_match.str().substr(1)) - _centerY;
         }

         return gcodeLayer;
     }

     const bool inwardCone = _config.opt_bool("inward_cone");

     double x_new = 0, y_new = 0;
     double z_layer = height;
     bool update_x = false, update_y = false;
     int c = inwardCone ? 1 : -1;

     std::vector<std::string> new_data;
     std::istringstream gcode_stream(gcodeLayer);
     std::string row;

     while (std::getline(gcode_stream, row)) {
         std::smatch g_match;
         if (!std::regex_search(row, g_match, _patternG)) {
             new_data.push_back(row);
             continue;
         }

         size_t semicolon_pos = row.find(';');
         std::string comment = "";
         if (semicolon_pos != std::string::npos) {
             comment = row.substr(semicolon_pos);
             row = row.substr(0, semicolon_pos);
         }

         std::smatch x_match, y_match, z_match;
         bool x_found = std::regex_search(row, x_match, _patternX);
         bool y_found = std::regex_search(row, y_match, _patternY);
         bool z_found = std::regex_search(row, z_match, _patternZ);

         if (!x_found && !y_found && !z_found) {
             new_data.push_back(row + comment);
             continue;
         }

         if (x_found) {
             x_new = std::stod(x_match.str().substr(1)) - _centerX;
             update_x = true;
         }
         if (y_found) {
             y_new = std::stod(y_match.str().substr(1)) - _centerY;
             update_y = true;
         }

         std::smatch e_match;
         bool e_found = std::regex_search(row, e_match, _patternE);
         double x_old_bt = _xOld * std::cos(_coneAngleRad), x_new_bt = x_new * std::cos(_coneAngleRad);
         double y_old_bt = _yOld * std::cos(_coneAngleRad), y_new_bt = y_new * std::cos(_coneAngleRad);
         double dist_transformed = std::sqrt(std::pow(x_new - _xOld, 2) + std::pow(y_new - _yOld, 2));

         int num_segm = static_cast<int>(dist_transformed / 0.5) + 1;
         std::vector<double> x_vals(num_segm + 1), y_vals(num_segm + 1), z_vals(num_segm + 1);
         for (int i = 0; i <= num_segm; ++i) {
             x_vals[i] = x_old_bt + i * (x_new_bt - x_old_bt) / num_segm + _centerX;
             y_vals[i] = y_old_bt + i * (y_new_bt - y_old_bt) / num_segm + _centerY;
         }

         if (inwardCone && !e_found && (update_x || update_y)) {
             double z_start = z_layer + c * std::sqrt(x_old_bt * x_old_bt + y_old_bt * y_old_bt) * std::tan(_coneAngleRad);
             double z_end = z_layer + c * std::sqrt(x_new_bt * x_new_bt + y_new_bt * y_new_bt) * std::tan(_coneAngleRad);
             for (int i = 0; i <= num_segm; ++i) {
                 z_vals[i] = z_start + i * (z_end - z_start) / num_segm;
             }
         } else {
             for (int i = 0; i <= num_segm; ++i) {
                 z_vals[i] = z_layer + c *
                     std::sqrt((x_vals[i] - _centerX) * (x_vals[i] - _centerX) +
                         (y_vals[i] - _centerY) * (y_vals[i] - _centerY)) *
                     std::tan(_coneAngleRad);
             }
             if (e_found) {
                 _zMax = std::max(_zMax, *std::max_element(z_vals.begin(), z_vals.end()));
             } else {
                 for (auto &z : z_vals) {
                     z = std::min(z, _zMax + 1);
                 }
             }
         }

         std::vector<double> distances_transformed(num_segm, dist_transformed / num_segm);
         std::vector<double> distances_bt(num_segm);
         for (int i = 1; i <= num_segm; ++i) {
             distances_bt[i - 1] = std::sqrt(std::pow(x_vals[i] - x_vals[i - 1], 2) + std::pow(y_vals[i] - y_vals[i - 1], 2) +
                 std::pow(z_vals[i] - z_vals[i - 1], 2));
         }

         std::string row_new = insertZ(row, z_vals[0]);
         row_new = replaceE(row_new, num_segm, 1, std::cos(_coneAngleRad));

         std::string replacement_rows;
         for (int j = 0; j < num_segm; ++j) {
             std::string single_row = std::regex_replace(row_new, _patternX, "X" + std::to_string(round(x_vals[j + 1] * 1000.0) / 1000.0));
             single_row = std::regex_replace(single_row, _patternY, "Y" + std::to_string(round(y_vals[j + 1] * 1000.0) / 1000.0));
             single_row = std::regex_replace(single_row, _patternZ,
                 "Z" + std::to_string(z_vals[j + 1] < _planarHeight ? z_layer :
                     round(z_vals[j + 1] * 1000.0) / 1000.0));
             single_row = replaceE(single_row, distances_transformed[j], distances_bt[j], 1);
             replacement_rows += single_row + "\n";
         }

         if (update_x) {
             _xOld = x_new;
             update_x = false;
         }
         if (update_y) {
             _yOld = y_new;
             update_y = false;
         }

         new_data.push_back(replacement_rows + comment);
     }

     std::ostringstream oss;
     for (const auto &line : new_data) {
         oss << line << "\n";
     }

     return oss.str();
 }

 std::string ConicalTransform::insertZ(const std::string &row, double zValue) const
 {
     std::smatch matchX, matchY, matchZ;

     std::string rowNew = row;
     if (std::regex_search(row, matchZ, _patternZ)) {
         rowNew = std::regex_replace(row, _patternZ, " Z" + std::to_string(round(zValue * 1000.0) / 1000.0));
     } else if (std::regex_search(row, matchY, _patternY)) {
         rowNew.insert(matchY.position() + matchY.length(), " Z" + std::to_string(round(zValue * 1000.0) / 1000.0));
     } else if (std::regex_search(row, matchX, _patternX)) {
         rowNew.insert(matchX.position() + matchX.length(), " Z" + std::to_string(round(zValue * 1000.0) / 1000.0));
     } else {
         rowNew = "Z" + std::to_string(round(zValue * 1000.0) / 1000.0) + " " + row;
     }
     return rowNew;
 }

 std::string ConicalTransform::replaceE(const std::string &row, double distOld, double distNew, double corrValue) const
 {
     std::smatch matchE;
     std::string rowNew = row;
     if (std::regex_search(row, matchE, _patternE)) {
         double eValOld = std::stod(matchE.str().substr(1));
         double eValNew = (distOld == 0) ? 0 : eValOld * distNew * corrValue / distOld;
         std::ostringstream oss;
         oss << "E" << std::fixed << std::setprecision(5) << eValNew;
         rowNew.replace(matchE.position(), matchE.length(), oss.str());
     }
     return rowNew;
 }

 double ConicalTransform::computeRadialAngle(double xOld, double yOld, double xNew, double yNew, bool inwardCone) const
 {
     double angle = std::atan2(yNew, xNew);
     if (inwardCone) {
         angle += M_PI;
     }
     return angle;
 }

 std::vector<double> ConicalTransform::computeUValues(const std::vector<double> &angleArray) const
 {
     int angleCount = angleArray.size();
     std::vector<std::vector<double>> angleCandidates(angleCount, std::vector<double>(21));
     for (int i = 0; i < angleCount; ++i) {
         for (int k = -10; k <= 10; ++k) {
             angleCandidates[i][k + 10] = std::round((angleArray[i] + k * 2 * M_PI) * 10000.0) / 10000.0;
         }
     }

     std::vector<double> angleInsert = {angleArray[0]};
     for (int i = 1; i < angleCount; ++i) {
         double anglePrev = angleInsert.back();
         double minDiff = std::abs(angleCandidates[i][0] - anglePrev);
         int idx = 0;
         for (int j = 1; j < 21; ++j) {
             double diff = std::abs(angleCandidates[i][j] - anglePrev);
             if (diff < minDiff) {
                 minDiff = diff;
                 idx = j;
             }
         }
         angleInsert.push_back(angleCandidates[i][idx]);
     }

     for (auto &angle : angleInsert) {
         angle = std::round(angle * 360.0 / (2 * M_PI) * 100.0) / 100.0;
     }

     return angleInsert;
 }

 std::string ConicalTransform::insertU(const std::string &row, double angle) const
 {
     std::smatch matchZ, matchU;

     std::string rowNew = row;
     if (std::regex_search(row, matchU, _patternU)) {
         rowNew = std::regex_replace(row, _patternU, "U" + std::to_string(angle));
     } else if (std::regex_search(row, matchZ, _patternZ)) {
         rowNew.insert(matchZ.position() + matchZ.length(), " U" + std::to_string(angle));
     }

     return rowNew;
 }

} // namespace Slic3r
