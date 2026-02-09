# # bash run_forzhinan.sh > run.txt
for i in {0..28}
do
   # 运行 main.py 并传递 --times 参数
   python3 main.py --skip_times "$i" --scenes 'bCPU9suPUw9' > "record/$i.txt"
done
# for i in {0..28}
# do
#    # 运行 main.py 并传递 --times 参数
#    python3 main.py --skip_times "$i" --scenes 'ziup5kvtCCR'
# done
# for i in {0..28}
# do
#    # 运行 main.py 并传递 --times 参数
#    python3 main.py --skip_times "$i" --scenes 'zt1RVoi7PcG'
# done
# for i in {0..28}
# do
#    # 运行 main.py 并传递 --times 参数
#    python3 main.py --skip_times "$i" --scenes 'VBzV5z6i1WS'
# done