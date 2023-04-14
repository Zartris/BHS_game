#pragma once

#include "torch/torch.h"

/*
 * This file contains aliases for the different tensor types used in the project.
 * It is purely made to make the code more readable and needs to be updated if the tensor types change.
 * The aliases are used in the following way:
 * TScalarDouble is a single value tensor of type double
 * Tensor2Double is a tensor of size 2 of type double
 * Tensor3Double is a tensor of size 3 of type double
 * TensorXDouble is a tensor of any size of type double
 */

using TScalarDouble = torch::Tensor;
using Tensor2Double = torch::Tensor;
using Tensor3Double = torch::Tensor;
using TensorXDouble = torch::Tensor;

using TScalarFloat = torch::Tensor;
using Tensor2Float = torch::Tensor;
using Tensor3Float = torch::Tensor;
using TensorXFloat = torch::Tensor;

using TScalarInt = torch::Tensor;
using Tensor2Int = torch::Tensor;
using Tensor3Int = torch::Tensor;
using TensorXInt = torch::Tensor;

using TScalarLong = torch::Tensor;
using Tensor2Long = torch::Tensor;
using Tensor3Long = torch::Tensor;
using TensorXLong = torch::Tensor;

static torch::TensorOptions TOptions(torch::ScalarType dtype, torch::Device d) {
    return torch::TensorOptions().dtype(dtype).device(d);
}