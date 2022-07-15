class ReLU(Module):
  __parameters__ = []
  training : bool
  def forward(self: __torch__.torch.nn.modules.activation.___torch_mangle_55.ReLU,
    argument_1: Tensor) -> Tensor:
    return torch.relu_(argument_1)
