import torch
import torch.nn as nn
import torch.nn.functional as F
from timm.models.layers import DropPath, trunc_normal_

import sys
sys.path.append('..')

# import pointnet2.pointnet2.pointnet2_utils as pt_utils
# from knn_cuda import KNN

# from .pointnet2_utils import PointNetFeaturePropagation, farthest_point_sample, index_points
from pointnet2_utils_ram import PointNetFeaturePropagation, farthest_point_sample, index_points


def knn(pc, center, n_neighbors=32):  
    dist = torch.cdist(center, pc) 
    neigbhors = dist.topk(k=n_neighbors,dim=2,largest=False) 
    return neigbhors.indices

def fps(data, number):
    '''
        data B N 3
        number int
    '''
    # fps_idx = pt_utils.furthest_point_sample(data, number)
    # fps_data = pt_utils.gather_operation(data.transpose(1, 2).contiguous(), fps_idx).transpose(1, 2).contiguous()
    # return fps_data

    fps_idx = farthest_point_sample(data, number)
    return index_points(data, fps_idx)

class FlashSelfAttention(nn.Module):
    """Implement the scaled dot product attention with softmax.
    Arguments
    ---------
        softmax_scale: The temperature to use for the softmax attention.
                      (default: 1/sqrt(d_keys) where d_keys is computed at
                      runtime)
        attention_dropout: The dropout rate to apply to the attention
                           (default: 0.0)
    """

    def __init__(
        self,
        causal=False,
        softmax_scale=None,
        attention_dropout=0.0,
        window_size=(-1, -1),
        alibi_slopes=None,
        deterministic=False,
    ):
        super().__init__()
        assert flash_attn_varlen_qkvpacked_func is not None, "FlashAttention is not installed"
        assert flash_attn_qkvpacked_func is not None, "FlashAttention is not installed"
        self.causal = causal
        self.softmax_scale = softmax_scale
        self.drop = nn.Dropout(attention_dropout)
        self.register_buffer("alibi_slopes", alibi_slopes, persistent=False)
        self.window_size = window_size
        self.deterministic = deterministic

    def forward(self, qkv, causal=None, cu_seqlens=None, max_seqlen=None):
        """Implements the multihead softmax attention.
        Arguments
        ---------
            qkv: The tensor containing the query, key, and value.
                If cu_seqlens is None and max_seqlen is None, then qkv has shape (B, S, 3, H, D).
                If cu_seqlens is not None and max_seqlen is not None, then qkv has shape
                (total, 3, H, D), where total is the sum of the sequence lengths in the batch.
            causal: if passed, will override self.causal
            cu_seqlens: (batch_size + 1,), dtype torch.int32. The cumulative sequence lengths
                of the sequences in the batch, used to index into qkv.
            max_seqlen: int. Maximum sequence length in the batch.
        Returns:
        --------
            out: (total, H, D) if cu_seqlens is not None and max_seqlen is not None,
                else (B, S, H, D).
        """
        assert qkv.dtype in [torch.float16, torch.bfloat16]
        assert qkv.is_cuda
        causal = self.causal if causal is None else causal
        unpadded = cu_seqlens is not None
        if self.alibi_slopes is not None:
            self.alibi_slopes = self.alibi_slopes.to(torch.float32)
        if unpadded:
            assert cu_seqlens.dtype == torch.int32
            assert max_seqlen is not None
            assert isinstance(max_seqlen, int)
            return flash_attn_varlen_qkvpacked_func(
                qkv,
                cu_seqlens,
                max_seqlen,
                self.drop.p if self.training else 0.0,
                softmax_scale=self.softmax_scale,
                causal=causal,
                alibi_slopes=self.alibi_slopes,
                window_size=self.window_size,
                deterministic=self.deterministic,
            )
        else:
            return flash_attn_qkvpacked_func(
                qkv,
                self.drop.p if self.training else 0.0,
                softmax_scale=self.softmax_scale,
                causal=causal,
                alibi_slopes=self.alibi_slopes,
                window_size=self.window_size,
                deterministic=self.deterministic,
            )

class Group(nn.Module):
    def __init__(self, num_group, group_size):
        super().__init__()
        self.num_group = num_group
        self.group_size = group_size
        # self.knn = KNN(k=self.group_size, transpose_mode=True)

    def forward(self, xyz):
        '''
            input: B N 3
            ---------------------------
            output: B G M 3
            center : B G 3
        '''
        batch_size, num_points, _ = xyz.shape
        # fps the centers out
        center = fps(xyz, self.num_group)  # B G 3
        # knn to get the neighborhood
        # _, idx = self.knn(xyz, center)  # B G M

        idx = knn(xyz, center, self.group_size)

        assert idx.size(1) == self.num_group
        assert idx.size(2) == self.group_size
        idx_base = torch.arange(0, batch_size, device=xyz.device).view(-1, 1, 1) * num_points
        idx = idx + idx_base
        idx = idx.view(-1)
        neighborhood = xyz.view(batch_size * num_points, -1)[idx, :]
        neighborhood = neighborhood.view(batch_size, self.num_group, self.group_size, 3).contiguous()
        # normalize
        neighborhood = neighborhood - center.unsqueeze(2)
        return neighborhood, center

class Encoder(nn.Module):
    def __init__(self, encoder_channel):
        super().__init__()
        self.encoder_channel = encoder_channel
        self.first_conv = nn.Sequential(
            nn.Conv1d(3, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.Conv1d(128, 256, 1)
        )
        self.second_conv = nn.Sequential(
            nn.Conv1d(512, 512, 1),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Conv1d(512, self.encoder_channel, 1)
        )

    def forward(self, point_groups):
        '''
            point_groups : B G N 3
            -----------------
            feature_global : B G C
        '''
        bs, g, n, _ = point_groups.shape
        point_groups = point_groups.reshape(bs * g, n, 3)
        # encoder
        feature = self.first_conv(point_groups.transpose(2, 1))
        feature_global = torch.max(feature, dim=2, keepdim=True)[0]
        feature = torch.cat([feature_global.expand(-1, -1, n), feature], dim=1)
        feature = self.second_conv(feature)
        feature_global = torch.max(feature, dim=2, keepdim=False)[0]
        return feature_global.reshape(bs, g, self.encoder_channel)

class Mlp(nn.Module):
    def __init__(self, in_features, hidden_features=None, out_features=None, act_layer=nn.GELU, drop=0.):
        super().__init__()
        out_features = out_features or in_features
        hidden_features = hidden_features or in_features
        self.fc1 = nn.Linear(in_features, hidden_features)
        self.act = act_layer()
        self.fc2 = nn.Linear(hidden_features, out_features)
        self.drop = nn.Dropout(drop)

    def forward(self, x):
        x = self.fc1(x)
        x = self.act(x)
        x = self.drop(x)
        x = self.fc2(x)
        x = self.drop(x)
        return x

class Attention(nn.Module):
    def __init__(self, dim, num_heads=8, qkv_bias=False, qk_scale=None, attn_drop=0., proj_drop=0.):
        super().__init__()
        self.num_heads = num_heads
        head_dim = dim // num_heads
        self.scale = qk_scale or head_dim ** -0.5

        self.qkv = nn.Linear(dim, dim * 3, bias=qkv_bias)
        self.attn_drop = nn.Dropout(attn_drop)
        self.proj = nn.Linear(dim, dim)
        self.proj_drop = nn.Dropout(proj_drop)

        # self.flash_atten = FlashSelfAttention()

    def forward(self, x):
        B, N, C = x.shape

        qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, C // self.num_heads).permute(2, 0, 3, 1, 4)
        q, k, v = qkv[0], qkv[1], qkv[2]  # make torchscript happy (cannot use tensor as tuple)

        attn = (q * self.scale) @ k.transpose(-2, -1)
        attn = attn.softmax(dim=-1)
        attn = self.attn_drop(attn)
        x = (attn @ v).transpose(1, 2).reshape(B, N, C)

        # # use flash attention
        # qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, C // self.num_heads).half()
        # x = self.flash_atten(qkv).reshape(B, N, C)
        # x = x.float()

        x = self.proj(x)
        x = self.proj_drop(x)
        return x

class Block(nn.Module):
    def __init__(self, dim, num_heads, mlp_ratio=4., qkv_bias=False, qk_scale=None, drop=0., attn_drop=0.,
                 drop_path=0., act_layer=nn.GELU, norm_layer=nn.LayerNorm):
        super().__init__()
        self.norm1 = norm_layer(dim)
        self.drop_path = DropPath(drop_path) if drop_path > 0. else nn.Identity()
        self.norm2 = norm_layer(dim)
        mlp_hidden_dim = int(dim * mlp_ratio)
        self.mlp = Mlp(in_features=dim, hidden_features=mlp_hidden_dim, act_layer=act_layer, drop=drop)

        self.attn = Attention(
            dim, num_heads=num_heads, qkv_bias=qkv_bias, qk_scale=qk_scale, attn_drop=attn_drop, proj_drop=drop)

    def forward(self, x):
        x = x + self.drop_path(self.attn(self.norm1(x)))
        x = x + self.drop_path(self.mlp(self.norm2(x)))
        return x

class TransformerEncoder(nn.Module):
    """ Transformer Encoder without hierarchical structure
    """
    def __init__(self, embed_dim=768, depth=12, num_heads=12, mlp_ratio=4., qkv_bias=False, qk_scale=None,
                 drop_rate=0., attn_drop_rate=0., drop_path_rate=0., fetch_list = [3, 7, 11]):
        super().__init__()
        
        self.fetch_list = fetch_list
        self.blocks = nn.ModuleList([
            Block(
                dim=embed_dim, num_heads=num_heads, mlp_ratio=mlp_ratio, qkv_bias=qkv_bias, qk_scale=qk_scale,
                drop=drop_rate, attn_drop=attn_drop_rate,
                drop_path=drop_path_rate[i] if isinstance(drop_path_rate, list) else drop_path_rate
            )
            for i in range(depth)])

    def forward(self, x, pos):
        feature_list = []
        # fetch_idx = [3, 7, 11]
        for i, block in enumerate(self.blocks):
            if pos is None:
                x = block(x)
            else:
                x = block(x + pos)
            if i in self.fetch_list:
                feature_list.append(x)
        return feature_list

class MapPosTransformer(nn.Module):
    def __init__(self, depth=12, num_heads=4, group_size = 32, num_group = 128, \
        fetch_list = [3,7,11], output_dim = 768, use_pose_embed = True, fix_pose_embed = False):
        super().__init__()

        self.trans_dim = 384
        self.depth = depth
        self.drop_path_rate = 0.1
        self.num_heads = num_heads
        self.fetch_list = fetch_list
        self.use_pose_embed = use_pose_embed
        self.fix_pose_embed = fix_pose_embed

        self.group_size = group_size
        self.num_group = num_group
        # grouper
        self.group_divider = Group(num_group=self.num_group, group_size=self.group_size)
        # define the encoder
        self.encoder_dims = 384
        self.encoder = Encoder(encoder_channel=self.encoder_dims)
        # bridge encoder and transformer

        if self.use_pose_embed and self.fix_pose_embed:
            self.pos_embed = nn.Sequential(
                nn.Linear(3, 128),
                nn.GELU(),
                nn.Linear(128, self.trans_dim)
            )

        dpr = [x.item() for x in torch.linspace(0, self.drop_path_rate, self.depth)]
        self.blocks = TransformerEncoder(
            embed_dim=self.trans_dim,
            depth=self.depth,
            drop_path_rate=dpr,
            num_heads=self.num_heads,
            fetch_list = self.fetch_list
        )

        self.norm = nn.LayerNorm(self.trans_dim)

        self.propagation_0 = PointNetFeaturePropagation(in_channel=384*(len(self.fetch_list)) + 3,
                                                        mlp=[self.trans_dim * 4, 1024])

        self.convs1 = nn.Conv1d(1024+384*(len(self.fetch_list))*2, 512, 1)
        self.convs2 = nn.Conv1d(512, 512, 1)
        self.convs3 = nn.Conv1d(512, output_dim, 1)
        self.bns1 = nn.BatchNorm1d(512)
        self.bns2 = nn.BatchNorm1d(512)

        self.relu = nn.ReLU()
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            trunc_normal_(m.weight, std=.02)
            if isinstance(m, nn.Linear) and m.bias is not None:
                nn.init.constant_(m.bias, 0)
        elif isinstance(m, nn.LayerNorm):
            nn.init.constant_(m.bias, 0)
            nn.init.constant_(m.weight, 1.0)
        elif isinstance(m, nn.Conv1d):
            trunc_normal_(m.weight, std=.02)
            if m.bias is not None:
                nn.init.constant_(m.bias, 0)

    def init_from_pretrain(self, pretrain_path):
        if pretrain_path is not None:
            ckpt = torch.load(pretrain_path)
            base_ckpt = {k.replace("module.", ""): v for k, v in ckpt['base_model'].items()}

            for k in list(base_ckpt.keys()):
                if k.startswith('MAE_encoder'):
                    base_ckpt[k[len('MAE_encoder.'):]] = base_ckpt[k]
                    del base_ckpt[k]
                elif k.startswith('base_model'):
                    base_ckpt[k[len('base_model.'):]] = base_ckpt[k]
                    del base_ckpt[k]

            incompatible = self.load_state_dict(base_ckpt, strict=False)
            print(f'Successful Loading the pre-trained point cloud Transformer from {pretrain_path}')

    def forward(self, pts, neighborhood = None, center = None, pos = None):
        B, N, C = pts.shape

        # divide the point cloud in the same form. This is important

        if neighborhood == None or center == None:
            # print("[Note] the input arguments 'neighborhood' or 'center' is None")
            neighborhood, center = self.group_divider(pts)

        group_input_tokens = self.encoder(neighborhood)  # B G N

        if self.use_pose_embed:
            if self.fix_pose_embed:
                pos = self.pos_embed(center)
            else:
                if pos is None:
                    raise Exception("Sorry, a valid position embedding should be given")
                    return

        # final input
        x = group_input_tokens
        # transformer
        feature_list = self.blocks(x, pos)

        feature_list = [self.norm(x).transpose(-1, -2).contiguous() for x in feature_list]
        x = torch.cat(feature_list, dim=1)

        # x = torch.cat((feature_list[0],feature_list[1],feature_list[2]), dim=1) #1152
        x_max = torch.max(x,2)[0]
        x_avg = torch.mean(x,2)
        x_max_feature = x_max.view(B, -1).unsqueeze(-1).repeat(1, 1, N)
        x_avg_feature = x_avg.view(B, -1).unsqueeze(-1).repeat(1, 1, N)
        x_global_feature = torch.cat((x_max_feature, x_avg_feature), 1) #1152*2

        f_level_0 = self.propagation_0(pts.transpose(-1, -2), center.transpose(-1, -2), pts.transpose(-1, -2), x)

        x = torch.cat((f_level_0, x_global_feature), 1)
        x = self.relu(self.bns1(self.convs1(x)))
        x = self.relu(self.bns2(self.convs2(x)))
        x = self.convs3(x)
        return x

class PointCloudTransformer(nn.Module):
    def __init__(self, depth=12, num_heads=4, group_size = 32, num_group = 128, fetch_list = [3,7,11], use_pose_embed = True, fix_pose_embed = False):
        super().__init__()

        self.trans_dim = 384
        self.depth = depth
        self.drop_path_rate = 0.1
        self.num_heads = num_heads
        self.fetch_list = fetch_list
        self.use_pose_embed = use_pose_embed
        self.fix_pose_embed = fix_pose_embed

        self.group_size = group_size
        self.num_group = num_group
        # grouper
        self.group_divider = Group(num_group=self.num_group, group_size=self.group_size)
        # define the encoder
        self.encoder_dims = 384
        self.encoder = Encoder(encoder_channel=self.encoder_dims)
        # bridge encoder and transformer

        if self.use_pose_embed and self.fix_pose_embed:
            self.pos_embed = nn.Sequential(
                nn.Linear(3, 128),
                nn.GELU(),
                nn.Linear(128, self.trans_dim)
            )

        dpr = [x.item() for x in torch.linspace(0, self.drop_path_rate, self.depth)]
        self.blocks = TransformerEncoder(
            embed_dim=self.trans_dim,
            depth=self.depth,
            drop_path_rate=dpr,
            num_heads=self.num_heads,
            fetch_list = self.fetch_list
        )

        self.norm = nn.LayerNorm(self.trans_dim)

        self.propagation_0 = PointNetFeaturePropagation(in_channel=384*(len(self.fetch_list)) + 3,
                                                        mlp=[self.trans_dim * 4, 1024])

        self.convs1 = nn.Conv1d(1024+384*(len(self.fetch_list))*2, 512, 1)
        self.convs2 = nn.Conv1d(512, 384, 1)
        self.convs3 = nn.Conv1d(384, 384, 1)
        self.bns1 = nn.BatchNorm1d(512)
        self.bns2 = nn.BatchNorm1d(384)

        self.relu = nn.ReLU()
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            trunc_normal_(m.weight, std=.02)
            if isinstance(m, nn.Linear) and m.bias is not None:
                nn.init.constant_(m.bias, 0)
        elif isinstance(m, nn.LayerNorm):
            nn.init.constant_(m.bias, 0)
            nn.init.constant_(m.weight, 1.0)
        elif isinstance(m, nn.Conv1d):
            trunc_normal_(m.weight, std=.02)
            if m.bias is not None:
                nn.init.constant_(m.bias, 0)

    def init_from_pretrain(self, pretrain_path):
        if pretrain_path is not None:
            ckpt = torch.load(pretrain_path)
            base_ckpt = {k.replace("module.", ""): v for k, v in ckpt['base_model'].items()}

            for k in list(base_ckpt.keys()):
                if k.startswith('MAE_encoder'):
                    base_ckpt[k[len('MAE_encoder.'):]] = base_ckpt[k]
                    del base_ckpt[k]
                elif k.startswith('base_model'):
                    base_ckpt[k[len('base_model.'):]] = base_ckpt[k]
                    del base_ckpt[k]

            incompatible = self.load_state_dict(base_ckpt, strict=False)
            print(f'Successful Loading the pre-trained point cloud Transformer from {pretrain_path}')

    def forward(self, pts, neighborhood = None, center = None, pos = None):
        B, N, C = pts.shape

        # divide the point cloud in the same form. This is important

        if neighborhood == None or center == None:
            # print("[Note] the input arguments 'neighborhood' or 'center' is None")
            neighborhood, center = self.group_divider(pts)

        group_input_tokens = self.encoder(neighborhood)  # B G N

        if self.use_pose_embed:
            if self.fix_pose_embed:
                pos = self.pos_embed(center)
            else:
                if pos is None:
                    raise Exception("Sorry, a valid position embedding should be given")
                    return

        # final input
        x = group_input_tokens
        # transformer
        feature_list = self.blocks(x, pos)

        feature_list = [self.norm(x).transpose(-1, -2).contiguous() for x in feature_list]
        x = torch.cat(feature_list, dim=1)

        # x = torch.cat((feature_list[0],feature_list[1],feature_list[2]), dim=1) #1152
        x_max = torch.max(x,2)[0]
        x_avg = torch.mean(x,2)
        x_max_feature = x_max.view(B, -1).unsqueeze(-1).repeat(1, 1, N)
        x_avg_feature = x_avg.view(B, -1).unsqueeze(-1).repeat(1, 1, N)
        x_global_feature = torch.cat((x_max_feature, x_avg_feature), 1) #1152*2

        f_level_0 = self.propagation_0(pts.transpose(-1, -2), center.transpose(-1, -2), pts.transpose(-1, -2), x)

        x = torch.cat((f_level_0, x_global_feature), 1)
        x = self.relu(self.bns1(self.convs1(x)))
        x = self.relu(self.bns2(self.convs2(x)))
        x = self.convs3(x)
        return x

# Point Cloud Transformer for point cloud after FPS
# without upsample propogation
class PointCloudTransformer2(nn.Module):
    def __init__(self, depth=12, num_heads=4, group_size = 32, num_group = 128, fetch_list = [3,7,11], use_pose_embed = True, fix_pose_embed = False):
        super().__init__()

        self.trans_dim = 384
        self.depth = depth
        self.drop_path_rate = 0.1
        self.num_heads = num_heads
        self.fetch_list = fetch_list
        self.use_pose_embed = use_pose_embed
        self.fix_pose_embed = fix_pose_embed

        self.group_size = group_size
        self.num_group = num_group
        # grouper
        self.group_divider = Group(num_group=self.num_group, group_size=self.group_size)
        # define the encoder
        self.encoder_dims = 384
        self.encoder = Encoder(encoder_channel=self.encoder_dims)
        # bridge encoder and transformer

        if self.use_pose_embed and self.fix_pose_embed:
            self.pos_embed = nn.Sequential(
                nn.Linear(3, 128),
                nn.GELU(),
                nn.Linear(128, self.trans_dim)
            )

        dpr = [x.item() for x in torch.linspace(0, self.drop_path_rate, self.depth)]
        self.blocks = TransformerEncoder(
            embed_dim=self.trans_dim,
            depth=self.depth,
            drop_path_rate=dpr,
            num_heads=self.num_heads,
            fetch_list = self.fetch_list
        )

        self.norm = nn.LayerNorm(self.trans_dim)

        # self.propagation_0 = PointNetFeaturePropagation(in_channel=384*(len(self.fetch_list)) + 3,
        #                                                 mlp=[self.trans_dim * 4, 1024])
        
        # self.mlp = nn.Sequential(
        #     nn.Conv1d(384*(len(self.fetch_list)), self.trans_dim * 4, 1),
        #     nn.BatchNorm1d(self.trans_dim * 4),
        #     nn.GELU(),
        #     nn.Conv1d(self.trans_dim * 4, 1024, 1),
        #     nn.BatchNorm1d(1024),
        #     nn.GELU()
        # )

        self.convs1 = nn.Conv1d(384*(len(self.fetch_list))*3, 512, 1)
        self.convs2 = nn.Conv1d(512, 256, 1)
        self.convs3 = nn.Conv1d(256, 64, 1)
        self.bns1 = nn.BatchNorm1d(512)
        self.bns2 = nn.BatchNorm1d(256)

        self.relu = nn.ReLU()
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            trunc_normal_(m.weight, std=.02)
            if isinstance(m, nn.Linear) and m.bias is not None:
                nn.init.constant_(m.bias, 0)
        elif isinstance(m, nn.LayerNorm):
            nn.init.constant_(m.bias, 0)
            nn.init.constant_(m.weight, 1.0)
        elif isinstance(m, nn.Conv1d):
            trunc_normal_(m.weight, std=.02)
            if m.bias is not None:
                nn.init.constant_(m.bias, 0)

    def init_from_pretrain(self, pretrain_path):
        if pretrain_path is not None:
            ckpt = torch.load(pretrain_path)
            base_ckpt = {k.replace("module.", ""): v for k, v in ckpt['base_model'].items()}

            for k in list(base_ckpt.keys()):
                if k.startswith('MAE_encoder'):
                    base_ckpt[k[len('MAE_encoder.'):]] = base_ckpt[k]
                    del base_ckpt[k]
                elif k.startswith('base_model'):
                    base_ckpt[k[len('base_model.'):]] = base_ckpt[k]
                    del base_ckpt[k]

            incompatible = self.load_state_dict(base_ckpt, strict=False)
            print(f'Successful Loading the pre-trained point cloud Transformer from {pretrain_path}')

    def forward(self, pts, neighborhood = None, center = None, pos = None):
        B, N, C = pts.shape

        # divide the point cloud in the same form. This is important

        if neighborhood == None or center == None:
            print("[Note] the input arguments 'neighborhood' or 'center' is None")
            neighborhood, center = self.group_divider(pts)
        
        B2, N2, C2 = center.shape

        group_input_tokens = self.encoder(neighborhood)  # B G N

        if self.use_pose_embed:
            if self.fix_pose_embed:
                pos = self.pos_embed(center)
            else:
                if pos is None:
                    raise Exception("Sorry, a valid position embedding should be given")
                    return

        # final input
        x = group_input_tokens
        # transformer
        feature_list = self.blocks(x, pos)

        feature_list = [self.norm(x).transpose(-1, -2).contiguous() for x in feature_list]
        x = torch.cat(feature_list, dim=1)

        # x = torch.cat((feature_list[0],feature_list[1],feature_list[2]), dim=1) #1152
        x_max = torch.max(x,2)[0]
        x_avg = torch.mean(x,2)
        x_max_feature = x_max.view(B, -1).unsqueeze(-1).repeat(1, 1, N2)
        x_avg_feature = x_avg.view(B, -1).unsqueeze(-1).repeat(1, 1, N2)
        x_global_feature = torch.cat((x_max_feature, x_avg_feature), 1) #1152*2

        # f_level_0 = self.propagation_0(pts.transpose(-1, -2), center.transpose(-1, -2), pts.transpose(-1, -2), x)
        # x = self.mlp(x)
        x = torch.cat((x, x_global_feature), 1)
        x = self.relu(self.bns1(self.convs1(x)))
        x = self.relu(self.bns2(self.convs2(x)))
        x = self.convs3(x)
        return x

if __name__ == '__main__':
    net = PointCloudTransformer(depth=8, num_heads=4, fetch_list = [3, 7], use_pose_embed = True, fix_pose_embed = True).cuda()
    pts = torch.rand(2, 1024, 3).cuda().contiguous()
    y = net(pts)
    print(y.shape)