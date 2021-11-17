module TrajectoryOptimization

using SparseArrays
using StaticArrays
using LinearAlgebra
using ForwardDiff
using Ipopt
using Requires
using Symbolics: jacobian_sparsity

# Utilities
include("Utilities/typeFlags.jl")
export Dynamics, Cost, Algebraic, AllFunctions # Function type flags 

# User function utilities
include("UserFunctionUtilities/PathFunction.jl")
include("UserFunctionUtilities/ADPathFunction.jl")
export ADPathFunction

# Conditionally load Snopt
function __init__()
    @require Snopt="0e9dc826-d618-11e8-1f57-c34e87fde2c0" include("NLPUtilities/snopt.jl")
end

end
