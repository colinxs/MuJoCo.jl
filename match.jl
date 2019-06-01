COMMENT=r"^//.*"
SECTION=r"^//-+\s[a-zA-Z]*\s-*"
iscomment(s) = occursin(COMMENT, s)
issection(s) = iscomment(s) && occursin(SECTION, s)

function partition(p, s)
  idxs = findall(p, s)
  prepend!(idxs, 1)
  push!(idxs, length(s))
  ranges = [idxs[(i-1)]:idxs[i] for i in 2:length(idxs)]
  return [s[r] for r in ranges]
end
    

