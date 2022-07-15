class NaiveSyncBatchNorm1d(Module):
  __parameters__ = ["weight", "bias", ]
  weight : Tensor
  bias : Tensor
  running_mean : Tensor
  running_var : Tensor
  num_batches_tracked : Tensor
  training : bool
  def forward(self: __torch__.mmdet3d.ops.norm.NaiveSyncBatchNorm1d,
    input: Tensor) -> Tensor:
    _0 = self.running_var
    _1 = self.running_mean
    _2 = self.bias
    _3 = torch.batch_norm(input, self.weight, _2, _1, _0, False, 0.01, 0.001, True)
    return _3
