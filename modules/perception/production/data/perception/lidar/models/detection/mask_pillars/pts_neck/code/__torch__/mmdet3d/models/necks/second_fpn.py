class SECONDFPN(Module):
  __parameters__ = []
  training : bool
  deblocks : __torch__.torch.nn.modules.container.___torch_mangle_125.ModuleList
  def forward(self: __torch__.mmdet3d.models.necks.second_fpn.SECONDFPN,
    argument_1: Tuple[Tensor, Tensor, Tensor]) -> List[Tensor]:
    _0 = getattr(self.deblocks, "2")
    _1 = getattr(self.deblocks, "1")
    _2 = getattr(self.deblocks, "0")
    input, input0, input1, = argument_1
    _3 = [(_2).forward(input, ), (_1).forward(input0, ), (_0).forward(input1, )]
    return [torch.cat(_3, 1)]
