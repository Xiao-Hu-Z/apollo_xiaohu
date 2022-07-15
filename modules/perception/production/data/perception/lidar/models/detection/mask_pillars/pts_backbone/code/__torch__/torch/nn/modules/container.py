class Sequential(Module):
  __parameters__ = []
  training : bool
  __annotations__["0"] = __torch__.torch.nn.modules.conv.Conv2d
  __annotations__["1"] = __torch__.mmdet3d.ops.norm.NaiveSyncBatchNorm2d
  __annotations__["2"] = __torch__.torch.nn.modules.activation.ReLU
  __annotations__["3"] = __torch__.torch.nn.modules.conv.___torch_mangle_13.Conv2d
  __annotations__["4"] = __torch__.mmdet3d.ops.norm.___torch_mangle_14.NaiveSyncBatchNorm2d
  __annotations__["5"] = __torch__.torch.nn.modules.activation.___torch_mangle_15.ReLU
  __annotations__["6"] = __torch__.torch.nn.modules.conv.___torch_mangle_16.Conv2d
  __annotations__["7"] = __torch__.mmdet3d.ops.norm.___torch_mangle_17.NaiveSyncBatchNorm2d
  __annotations__["8"] = __torch__.torch.nn.modules.activation.___torch_mangle_18.ReLU
  __annotations__["9"] = __torch__.torch.nn.modules.conv.___torch_mangle_19.Conv2d
  __annotations__["10"] = __torch__.mmdet3d.ops.norm.___torch_mangle_20.NaiveSyncBatchNorm2d
  __annotations__["11"] = __torch__.torch.nn.modules.activation.___torch_mangle_21.ReLU
  def forward(self: __torch__.torch.nn.modules.container.Sequential,
    input: Tensor) -> Tensor:
    _0 = getattr(self, "1")
    _1 = (getattr(self, "0")).forward(input, )
    _2 = getattr(self, "3")
    _3 = (getattr(self, "2")).forward((_0).forward(_1, ), )
    _4 = getattr(self, "5")
    _5 = (getattr(self, "4")).forward((_2).forward(_3, ), )
    _6 = getattr(self, "7")
    _7 = (getattr(self, "6")).forward((_4).forward(_5, ), )
    _8 = getattr(self, "9")
    _9 = (getattr(self, "8")).forward((_6).forward(_7, ), )
    _10 = getattr(self, "11")
    _11 = (getattr(self, "10")).forward((_8).forward(_9, ), )
    return (_10).forward(_11, )
