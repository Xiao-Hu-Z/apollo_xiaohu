class Sequential(Module):
  __parameters__ = []
  training : bool
  __annotations__["0"] = __torch__.torch.nn.modules.conv.___torch_mangle_22.Conv2d
  __annotations__["1"] = __torch__.mmdet3d.ops.norm.___torch_mangle_23.NaiveSyncBatchNorm2d
  __annotations__["2"] = __torch__.torch.nn.modules.activation.___torch_mangle_24.ReLU
  __annotations__["3"] = __torch__.torch.nn.modules.conv.___torch_mangle_25.Conv2d
  __annotations__["4"] = __torch__.mmdet3d.ops.norm.___torch_mangle_26.NaiveSyncBatchNorm2d
  __annotations__["5"] = __torch__.torch.nn.modules.activation.___torch_mangle_27.ReLU
  __annotations__["6"] = __torch__.torch.nn.modules.conv.___torch_mangle_28.Conv2d
  __annotations__["7"] = __torch__.mmdet3d.ops.norm.___torch_mangle_29.NaiveSyncBatchNorm2d
  __annotations__["8"] = __torch__.torch.nn.modules.activation.___torch_mangle_30.ReLU
  __annotations__["9"] = __torch__.torch.nn.modules.conv.___torch_mangle_31.Conv2d
  __annotations__["10"] = __torch__.mmdet3d.ops.norm.___torch_mangle_32.NaiveSyncBatchNorm2d
  __annotations__["11"] = __torch__.torch.nn.modules.activation.___torch_mangle_33.ReLU
  __annotations__["12"] = __torch__.torch.nn.modules.conv.___torch_mangle_34.Conv2d
  __annotations__["13"] = __torch__.mmdet3d.ops.norm.___torch_mangle_35.NaiveSyncBatchNorm2d
  __annotations__["14"] = __torch__.torch.nn.modules.activation.___torch_mangle_36.ReLU
  __annotations__["15"] = __torch__.torch.nn.modules.conv.___torch_mangle_37.Conv2d
  __annotations__["16"] = __torch__.mmdet3d.ops.norm.___torch_mangle_38.NaiveSyncBatchNorm2d
  __annotations__["17"] = __torch__.torch.nn.modules.activation.___torch_mangle_39.ReLU
  def forward(self: __torch__.torch.nn.modules.container.___torch_mangle_40.Sequential,
    argument_1: Tensor) -> Tensor:
    _0 = getattr(self, "1")
    _1 = (getattr(self, "0")).forward(argument_1, )
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
    _12 = getattr(self, "13")
    _13 = (getattr(self, "12")).forward((_10).forward(_11, ), )
    _14 = getattr(self, "15")
    _15 = (getattr(self, "14")).forward((_12).forward(_13, ), )
    _16 = getattr(self, "17")
    _17 = (getattr(self, "16")).forward((_14).forward(_15, ), )
    return (_16).forward(_17, )
