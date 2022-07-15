class ConvTranspose2d(Module):
  __parameters__ = ["weight", ]
  weight : Tensor
  training : bool
  def forward(self: __torch__.mmcv.cnn.bricks.wrappers.ConvTranspose2d,
    input: Tensor) -> Tensor:
    input0 = torch._convolution(input, self.weight, None, [1, 1], [0, 0], [1, 1], True, [0, 0], 1, False, False, True)
    return input0
