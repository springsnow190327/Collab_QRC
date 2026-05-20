/*
 * Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
 * property and proprietary rights in and to this material, related
 * documentation and any modifications thereto. Any use, reproduction,
 * disclosure or distribution of this material and related documentation
 * without an express license agreement from NVIDIA CORPORATION or
 * its affiliates is strictly prohibited.
 */

namespace pynvblox {

template <typename ElementType>
nvblox::ImageView<ElementType> view_from_tensor(torch::Tensor tensor_image) {
  CHECK(tensor_image.sizes().size() == 2 || tensor_image.sizes().size() == 3)
      << "Image tensor must have a dimension of either 2 (scalar image) or 3 "
         "(array image)";
  CHECK(tensor_image.is_contiguous())
      << "Only non-strided tensors are supported\n";
  CHECK(tensor_image.is_cuda()) << "Only CUDA tensors are supported\n";

  const int num_rows = tensor_image.sizes()[0];
  const int num_cols = tensor_image.sizes()[1];
  const int num_elements_per_pixel =
      (tensor_image.sizes().size() == 3) ? tensor_image.sizes()[2] : 1;

  CHECK_EQ(
      static_cast<size_t>(num_elements_per_pixel * tensor_image.element_size()),
      sizeof(ElementType))
      << "Element size mismatch";

  return nvblox::ImageView<ElementType>(
      num_rows, num_cols,
      reinterpret_cast<ElementType*>(tensor_image.data_ptr()));
}

template <typename ElementType>
nvblox::MaskedImageView<ElementType> masked_view_from_tensor(
    torch::Tensor tensor_image, std::optional<torch::Tensor> tensor_mask) {
  const nvblox::ImageView<ElementType> image_view =
      view_from_tensor<ElementType>(tensor_image);

  std::optional<nvblox::ImageView<const uint8_t>> mask_view = std::nullopt;
  if (tensor_mask.has_value()) {
    mask_view = view_from_tensor<const uint8_t>(tensor_mask.value());
  }

  return nvblox::MaskedImageView<ElementType>(image_view, mask_view);
}

}  // namespace pynvblox
