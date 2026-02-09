import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MAGNUM_API_IMPLEMENTATION"] = "EGL"
os.environ["MAGNUM_LOG"] = "quiet"
os.environ["HABITAT_SIM_LOG"] = "quiet"

from utils.arguments import get_args
import torch
import numpy as np
from episode import Episode


fileName = 'data/matterport_category_mappings.tsv'

text = ''
lines = []
items = []
hm3d_semantic_mapping={}
hm3d_semantic_index={}
hm3d_semantic_index_inv={}

with open(fileName, 'r') as f:
    text = f.read()
lines = text.split('\n')[1:]

for l in lines:
    items.append(l.split('    '))

def setup_seed(args):
    seed = args.seed
    torch.manual_seed(seed)
    if args.cuda:
        torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    torch.backends.cudnn.deterministic = True

if __name__ == '__main__':
    args = get_args()
    setup_seed(args)

    device = args.device = torch.device("cuda:0" if args.cuda else "cpu")
    # Starting environments
    torch.set_num_threads(1)
    torch.set_grad_enabled(False)

    # Start navigation
    navigation_episodes = Episode(args)
    navigation_episodes.start()