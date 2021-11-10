using TrajectoryOptimization
using Documenter

DocMeta.setdocmeta!(TrajectoryOptimization, :DocTestSetup, :(using TrajectoryOptimization); recursive=true)

makedocs(;
    modules=[TrajectoryOptimization],
    authors="Grant Hecht",
    repo="https://github.com/GrantHecht/TrajectoryOptimization.jl/blob/{commit}{path}#{line}",
    sitename="TrajectoryOptimization.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://GrantHecht.github.io/TrajectoryOptimization.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/GrantHecht/TrajectoryOptimization.jl",
    devbranch="main",
)
