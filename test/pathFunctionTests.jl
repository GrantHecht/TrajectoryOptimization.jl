using TrajectoryOptimization
using LinearAlgebra

# Simple keplarian dynamics with thrust acceleration ODE function
function keplar!(out, x, u, p, t)
    # Earth gravitational parameter (likely would never be a static parameter,
    # but using here for testing purposes...)
    μ = p[1]

    # Requirements
    @views r    = norm(x[1:3])

    # Derivatives
    @views out[1:3] .= x[4:6]
    @views out[4:6] .= -(μ / r^3).*x[1:3] .+ u
    return nothing
end

# Analytical Jacobians
function stateJac!(out, x, u, p, t)
    μ = p[1]
    out .= 0.0

    # ddr/dv 
    for i in 1:3
        out[i, 3 + i] = 1.0
    end

    # ddv/dr
    @views r = norm(x[1:3])
    @views mul!(out[4:6, 1:3], x[1:3], transpose(x[1:3]))
    @views out[4:6, 1:3] .*= (3.0 / r^2)
    for i in 1:3; out[3 + i, i] -= 1.0; end
    @views out[4:6, 1:3] .*= (μ/r^3)
    return nothing
end
function controlJac!(out, x, u, p, t)
    out .= 0.0
    for i in 1:3
        out[3 + i, i] = 1.0
    end
    return nothing
end
function staticJac!(out, x, u, p, t)
    out .= 0.0
    @views r = norm(x[1:3])
    @views out[4:6] .= -(1/r^3).*x[1:3]
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
# State Jacobian
stateJac        = Matrix{Float64}(undef, 6, 6)
stateJacWrapped = Matrix{Float64}(undef, 6, 6)
TrajectoryOptimization.EvaluateJacobian(TrajectoryOptimization.State(), 
    adpf, stateJac, states, controls, static, time)
stateJac!(stateJacWrapped, states, controls, static, time)
for col in 1:6
    for row in 1:6
        @test stateJac[row, col] ≈ stateJacWrapped[row, col]
    end
end

# Control Jacobian
controlJac      = Matrix{Float64}(undef, 6, 3)
controlJacWrap  = Matrix{Float64}(undef, 6, 3)
TrajectoryOptimization.EvaluateJacobian(TrajectoryOptimization.Control(),
    adpf, controlJac, states, controls, static, time)
controlJac!(controlJacWrap, states,  controls, static, time)
for col in 1:3
    for row in 1:6
        @test controlJac[row, col] ≈ controlJacWrap[row, col]
    end
end

# Static Jacobian
staticJac       = Matrix{Float64}(undef, 6, 1)
staticJacWrap   = Matrix{Float64}(undef, 6, 1)
TrajectoryOptimization.EvaluateJacobian(TrajectoryOptimization.Static(),
    adpf, staticJac, states, controls, static, time)
staticJac!(staticJacWrap, states, controls, static, time)
for row in 1:6
    @test staticJac[row, 1] ≈ staticJacWrap[row, 1]
end

# Time Jacobian
timeJac         = Matrix{Float64}(undef, 6, 1)
TrajectoryOptimization.EvaluateJacobian(TrajectoryOptimization.Time(),
    adpf, timeJac, states, controls, static, time)
for row in 1:6
    @test timeJac[row, 1] ≈ 0.0
end