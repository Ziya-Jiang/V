"""Offline exclusion semantics, crop coordinates and SAM3 state isolation."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
from PIL import Image

import cart_book_grasp_v2_1_bookends as ends


class FakeProcessor:
    def __init__(self):
        self.images=0;self.resets=0;self.calls=[]

    def set_image(self,image):
        self.images+=1
        self.shape=np.asarray(image).shape[:2]
        return dict(backbone_out='image embedding')

    def reset_all_prompts(self,state):
        self.resets+=1
        state.clear();state['backbone_out']='image embedding'

    def set_text_prompt(self,prompt,state):
        if 'scores' in state:raise AssertionError('Previous prompt was not cleared')
        self.calls.append(prompt)
        h,w=self.shape
        mask=np.zeros((1,1,h,w),bool);mask[:,:,2:h-2,3:w-3]=True
        state.update(masks=mask,scores=np.array([.6]),boxes=np.array([[3,2,w-3,h-2]]))
        return state


class BookendTests(unittest.TestCase):
    def setUp(self):
        self.c={'bookend_filter':copy.deepcopy(ends.DEFAULTS)}
        self.rgb=np.zeros((90,150,3),np.uint8)
        self.items=[]
        for x0,x1 in [(20,30),(40,65),(80,110)]:
            mask=np.zeros((90,150),bool);mask[20:70,x0:x1]=True
            self.items.append(dict(book_mask=mask,score=.9,box=[x0,20,x1,70]))

    def exclude(self,mask,score=.6):
        report={}
        kept=ends.exclude_bookends(self.items,{'bookend':[(score,mask,[0,0,150,90])]},self.c,report)
        return kept,report

    def test_unique_bookend_whole_candidate_removed_but_book_masks_unchanged(self):
        masks=[v['book_mask'].copy() for v in self.items]
        kept,report=self.exclude(masks[0])
        self.assertEqual(report['removed_count'],1)
        self.assertEqual(len(kept),2)
        # A book score higher than the bookend score is not a cross-class veto.
        for item,original in zip(self.items,masks):np.testing.assert_array_equal(item['book_mask'],original)
        self.assertIs(kept[0],self.items[1])

    def test_broad_or_partial_or_low_confidence_masks_do_not_delete_books(self):
        group=self.items[0]['book_mask']|self.items[1]['book_mask']
        partial=self.items[0]['book_mask'].copy();partial[40:]=False
        for mask,score in [(group,.99),(np.ones((90,150),bool),.99),(partial,.9),
                           (self.items[0]['book_mask'],.24)]:
            kept,report=self.exclude(mask,score)
            self.assertEqual(len(kept),3);self.assertEqual(report['removed_count'],0)

    def test_mask_covering_another_candidate_is_not_exclusive(self):
        # Dominant mask qualifies bidirectionally but also covers 30% of a neighbor.
        mask=self.items[2]['book_mask'].copy();mask[20:35,20:30]=True
        kept,report=self.exclude(mask)
        self.assertEqual(len(kept),3)
        self.assertEqual(report['evidence'][0]['decision'],'overlaps_other_book')

    def test_image_embedding_reused_and_prompt_state_reset(self):
        processor=FakeProcessor();checkpoint=Mock()
        result=ends.infer_prompts(processor,np.asarray,Image.fromarray(self.rgb),['bookend','book holder'],checkpoint)
        self.assertEqual(processor.images,1);self.assertEqual(processor.resets,2)
        self.assertGreaterEqual(checkpoint.call_count,4)
        result['bookend'][0][1][:]=False
        self.assertTrue(result['book holder'][0][1].any())

    def test_crop_restores_original_pixels_and_fingerprint_invalidates_cache(self):
        processor=FakeProcessor();factory=Mock(return_value=(processor,np.asarray,Image))
        with tempfile.TemporaryDirectory() as path:
            results,detail=ends.detect_bookends(self.rgb,self.items,self.c,factory,cache_dir=path)
            x0,y0,x1,y1=detail['crop_xyxy'];mask=results['bookend'][0][1]
            expected=np.zeros((90,150),bool);expected[y0+2:y1-2,x0+3:x1-3]=True
            np.testing.assert_array_equal(mask,expected)
            self.assertEqual(processor.images,1)
            again,detail=ends.detect_bookends(self.rgb,self.items,self.c,factory,cache_dir=path)
            self.assertTrue(detail['cache_hit']);self.assertEqual(processor.images,1)
            np.testing.assert_array_equal(again['bookend'][0][1],mask)
            self.rgb[y0:y1,x0:x1]=5
            ends.detect_bookends(self.rgb,self.items,self.c,factory,cache_dir=path)
            self.assertEqual(processor.images,2)

    def test_empty_invalid_and_wrong_prompt_cache(self):
        with tempfile.TemporaryDirectory() as path:
            cache=Path(path)/'cache.npz'
            ends.save_prompt_cache(cache,self.rgb,{'bookend':[]})
            self.assertEqual(ends.load_prompt_cache(cache,self.rgb,['bookend']),{'bookend':[]})
            self.assertIsNone(ends.load_prompt_cache(cache,self.rgb,['book holder']))
            cache.write_bytes(b'invalid')
            self.assertIsNone(ends.load_prompt_cache(cache,self.rgb,['bookend']))

    def test_disabled_or_empty_candidates_do_not_infer(self):
        factory=Mock(side_effect=AssertionError('SAM3 must not run'))
        self.c['bookend_filter']['enabled']=False
        raw,detail=ends.detect_bookends(self.rgb,self.items,self.c,factory)
        self.assertEqual(raw,{})
        self.c['bookend_filter']['enabled']=True
        raw,detail=ends.detect_bookends(self.rgb,[],self.c,factory)
        self.assertEqual(raw,{})

    def test_invalid_config_rejected(self):
        for key,value in [('min_score',float('nan')),('enabled',1),('prompts',['book']),
                           ('prompts',['bookend','bookend']),('crop_padding_fraction',-1)]:
            c=copy.deepcopy(self.c);c['bookend_filter'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):ends.filter_config(c)


if __name__=='__main__':unittest.main()
