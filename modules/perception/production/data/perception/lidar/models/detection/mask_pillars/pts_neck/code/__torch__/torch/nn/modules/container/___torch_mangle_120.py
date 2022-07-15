class Sequential(Module):
  __parameters__ = []
  training : bool
  __annotations__["0"] = __torch__.mmcv.cnn.bricks.wrappers.___torch_mangle_117.ConvTranspose2d
  __annotations__["1"] = __torch__.mmdet3d.ops.norm.___torch_mangle_118.NaiveSyncBatchNorm2d
  __annotations__["2"] = __torch__.torch.nn.modules.activation.___torch_mangle_119.ReLU
  def forward(self: __torch__.torch.nn.modules.container.___torch_mangle_120.Sequential,
    input: Tensor) -> Tensor:
    _0 = getattr(self, "1")
    _1 = (getattr(self, "0")).forward(input, )
    _2 = (getattr(self, "2")).forward((_0).forward(_1, ), )
    return _2
