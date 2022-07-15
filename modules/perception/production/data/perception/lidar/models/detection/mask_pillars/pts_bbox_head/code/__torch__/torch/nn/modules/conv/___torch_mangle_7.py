class Conv2d(Module):
  __parameters__ = ["weight", "bias", ]
  weight : Tensor
  bias : Tensor
  training : bool
  def forward(self: __torch__.torch.nn.modules.conv.___torch_mangle_7.Conv2d,
    input: Tensor) -> Tensor:
    _0 = self.bias
    dir_cls_preds = torch._convolution(input, self.weight, _0, [1, 1], [0, 0], [1, 1], False, [0, 0], 1, False, False, True)
    return dir_cls_preds
