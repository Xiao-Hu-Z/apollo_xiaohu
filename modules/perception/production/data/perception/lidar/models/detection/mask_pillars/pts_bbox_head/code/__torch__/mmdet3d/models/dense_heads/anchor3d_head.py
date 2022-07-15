class Anchor3DHead(Module):
  __parameters__ = []
  training : bool
  loss_cls : __torch__.mmdet.models.losses.focal_loss.FocalLoss
  loss_bbox : __torch__.mmdet.models.losses.smooth_l1_loss.SmoothL1Loss
  loss_dir : __torch__.mmdet.models.losses.cross_entropy_loss.CrossEntropyLoss
  conv_cls : __torch__.torch.nn.modules.conv.Conv2d
  conv_reg : __torch__.torch.nn.modules.conv.___torch_mangle_6.Conv2d
  conv_dir_cls : __torch__.torch.nn.modules.conv.___torch_mangle_7.Conv2d
  def forward(self: __torch__.mmdet3d.models.dense_heads.anchor3d_head.Anchor3DHead,
    argument_1: List[Tensor]) -> Tuple[Tensor, Tensor, Tensor]:
    _0 = self.conv_dir_cls
    _1 = self.conv_reg
    _2 = self.conv_cls
    input, = argument_1
    _3 = (_2).forward(input, )
    _4 = (_1).forward(input, )
    _5 = (_0).forward(input, )
    _6 = torch.contiguous(torch.permute(_3, [0, 2, 3, 1]), memory_format=0)
    _7 = torch.contiguous(torch.permute(_4, [0, 2, 3, 1]), memory_format=0)
    _8 = torch.contiguous(torch.permute(_5, [0, 2, 3, 1]), memory_format=0)
    return (_6, _7, _8)
