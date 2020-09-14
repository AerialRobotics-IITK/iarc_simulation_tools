# git diff --name-only HEAD~2
declare -a var
var=($(git diff --name-only HEAD~4))
# echo ${var[2]}

len=${#var[@]}
str="a"
echo "$str"
for((i=0;i<len;i++)); do
    cpplint --filter=-legal/copyright,-readability/multiline_comment,-readability/braces,-build/include_order,-build/c++11,-build/include_what_you_use,-runtime/string,-whitespace/indent,-whitespace/comments,+build/namespace,+readability/constructors --linelength=160 ${var[$i]}

    A="$(cut -d'/' -f1 <<<"${var[$i]}")"
    if [[ "$A" == *"iarc"* && "$A" != "$str" ]]; then
        # if [[ "$A" != "$str" ]]; then
            echo "$A"
            str="$A"
        # fi
    fi
done


declare -a pkgs

# for f in *; do
#   if [ -d $f ]
#   then
#     # echo "$f"
#     cd "$f"
#     # ls
#     dir="src/"
#     if [ -d "$dir" ]
#     then
#         # echo "$f"
#         cd src
#         for cf in *; do
#             # echo "The file $cf has the following corrections"
#             cpplint --filter=-legal/copyright,-readability/multiline_comment,-readability/braces,-build/include_order,-build/c++11,-build/include_what_you_use,-runtime/string,-whitespace/indent,-whitespace/comments,+build/namespace,+readability/constructors --linelength=160 $cf
#         done
#         cd ..
#     fi

#     dir2="include/"
#     if [ -d "$dir2" ]
#     then
#         cd include
#         cd "$f"
#         for hf in *; do
#             cpplint --filter=-legal/copyright,-readability/multiline_comment,-readability/braces,-build/include_order,-build/c++11,-build/include_what_you_use,-runtime/string,-whitespace/indent,-whitespace/comments,+build/namespace,+readability/constructors --linelength=160 $hf
#         done
#         cd ..
#         cd ..
#     fi      

#     cd ..
#   fi
# done