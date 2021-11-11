module TrajectoryOptimization

using SparseArrays, StaticArrays, LinearAlgebra
using ForwardDiff, Snopt
using Symbolics: jacobian_sparsity

# Utilities
include("Utilities/typeFlags.jl")

# User function utilities
include("UserFunctionUtilities/PathFunction.jl")
include("UserFunctionUtilities/ADPathFunction.jl")

end
