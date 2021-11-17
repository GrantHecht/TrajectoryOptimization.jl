using TrajectoryOptimization
using LinearAlgebra

# Simple keplarian dynamics function
function keplar!(out, x, u, p, t)
    # Earth gravitational parameter (likely would never be a static parameter,
    # but using here for testing purposes...)
    μ = p[1]

    # Requirements
    @views r    = norm(x[1:3])

    # Derivatives
    @views out[1:3] .= x[4:6]
    @views out[4:6] .= -(μ / r^3).*x[1:3] .+ u
end

# ===== Construct AD Path Function
nFuncs      = 6
nStates     = 6
nControls   = 3
nStatic     = 1
adpf        = ADPathFunction(Dynamics(), keplar!, nFuncs, nStates, nControls, nStatic)

# ===== Test function evaluation
# Create inputs 
out         = Vector{Float64}(undef, 
                TrajectoryOptimization.GetNumberOfFunctions(adpf))
states      = rand(TrajectoryOptimization.GetNumberOfStates(adpf))
controls    = rand(TrajectoryOptimization.GetNumberOfControls(adpf))
static      = rand(TrajectoryOptimization.GetNumberOfStatics(adpf))
time        = 0.0

# Test that nuber of parameters are correct
@test length(out)       == nFuncs
@test length(states)    == nStates
@test length(controls)  == nControls
@test length(static)    == nStatic

# Evaluate functions with method
TrajectoryOptimization.EvaluateFunction(adpf, out, states, controls, static, time)

# Evaluate wrapped function
outWrapped  = Vector{Float64}(undef, nFuncs)
keplar!(outWrapped, states, controls, static, time)

# Test for equality
for i in 1:nFuncs; @test out[i] == outWrapped[i]; end

# ===== Test Jacobian evaluation