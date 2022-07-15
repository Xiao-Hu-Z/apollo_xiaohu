class NaiveSyncBatchNorm2d(Module):
  __parameters__ = ["weight", "bias", ]
  weight : Tensor
  bias : Tensor
  running_mean : Tensor
  running_var : Tensor
  num_batches_tracked : Tensor
  training : bool
  def forward(self: __torch__.mmdet3d.ops.norm.___torch_mangle_54.NaiveSyncBatchNorm2d,
    argument_1: Tensor) -> Tensor:
    _0 = self.running_var
    _1 = self.running_mean
    _2 = self.bias
    input = torch.batch_norm(argument_1, self.weight, _2, _1, _0, False, 0.01, 0.001, True)
    return input
