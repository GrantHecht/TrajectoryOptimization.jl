
# Abstract path function
abstract type PathFunction{type <: FunctionType} end

function GetFunctionType(pf::PathFunction{type}) where {type}
    return type
end

function EvaluateFunction(pf::PathFunction,
                          out::AbstractVector,
                          state::AbstractVector,
                          control::AbstractVector,
                          static::AbstractVector,
                          time::AbstractFloat)
    pf.func!(out,state,control,static,time)
    return nothing
end

function GetNumberOfFunctions(pf::PathFunction)
    return pf.nFuncs
end

function GetNumberOfStates(pf::PathFunction)
    return pf.nStates
end

function GetNumberOfControls(pf::PathFunction)
    return pf.nControls
end

function GetNumberOfStatics(pf::PathFunction)
    return pf.nStatic
end

function SetFunctionNames!(pf::PathFunction, names::Vector{String})
    for i in 1:length(names)
        push!(pf.funcNames, name[i])
    end
    return nothing
end

function GetFunctionNames(pf::PathFunction)
    return pf.funcNames
end