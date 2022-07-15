class SECOND(Module):
  __parameters__ = []
  training : bool
  blocks : __torch__.torch.nn.modules.container.___torch_mangle_60.ModuleList
  def forward(self: __torch__.mmdet3d.models.backbones.second.SECOND,
    input: Tensor) -> Tuple[Tensor, Tensor, Tensor]:
    _0 = getattr(self.blocks, "2")
    _1 = getattr(self.blocks, "1")
    _2 = (getattr(self.blocks, "0")).forward(input, )
    _3 = (_1).forward(_2, )
    return (_2, _3, (_0).forward(_3, ))
