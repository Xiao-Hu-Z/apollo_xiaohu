class Linear(Module):
  __parameters__ = ["weight", ]
  weight : Tensor
  training : bool
  def forward(self: __torch__.torch.nn.modules.linear.Linear,
    inputs: Tensor) -> Tensor:
    x = torch.matmul(inputs, torch.t(self.weight))
    return x
