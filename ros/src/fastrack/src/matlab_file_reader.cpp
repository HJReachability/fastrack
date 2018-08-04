/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Utility for reading variables out of *.mat files.
//
///////////////////////////////////////////////////////////////////////////////

#include <fastrack/utils/matlab_file_reader.h>

namespace fastrack {

// Open a file. Return bool upon success.
bool MatlabFileReader::Open(const std::string& file_name) {
  // Close any open file before opening a new file.
  if (IsOpen()) Close();

  // Open the new file.
  mat_fp_ = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  return IsOpen();
}

// Read scalar. Returns bool indicating success.
bool MatlabFileReader::ReadScalar(const std::string& field_name,
                                  double* value) {
  // Make sure 'value' is non-null.
  if (!value) return false;

  // Extract this variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_DOUBLE) return false;

  *value = *static_cast<double*>(variable);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read scalar. Returns bool indicating success.
bool MatlabFileReader::ReadScalar(const std::string& field_name,
                                  size_t* value) {
  // Make sure 'value' is non-null.
  if (!value) return false;

  // Extract this variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_UINT64) return false;

  *value = *static_cast<size_t*>(variable);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read vector. Returns bool indicating success.
bool MatlabFileReader::ReadVector(const std::string& field_name,
                                  std::vector<double>* values) {
  // Make sure 'values' is non-null.
  if (!values) return false;
  values->clear();

  // Extract variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_DOUBLE) return false;

  size_t num_elements = variable->nbytes / variable->data_size;
  for (size_t ii = 0; ii < num_elements; ii++)
    values->emplace_back(static_cast<double*>(variable->data)[ii]);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

// Read vector. Returns bool indicating success.
bool MatlabFileReader::ReadVector(const std::string& field_name,
                                  std::vector<size_t>* values) {
  // Make sure 'values' is non-null.
  if (!values) return false;
  values->clear();

  // Extract variable and check type.
  matvar_t* variable = Mat_VarRead(mat_fp_, field_name.c_str());
  if (!variable || variable->data_type != MAT_T_UINT64) return false;

  size_t num_elements = variable->nbytes / variable->data_size;
  for (size_t ii = 0; ii < num_elements; ii++)
    values->emplace_back(static_cast<size_t*>(variable->data)[ii]);

  // Free variable.
  Mat_VarFree(variable);
  return true;
}

}  // namespace fastrack

#endif
