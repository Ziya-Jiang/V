import os,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from cart_book_grasp_v2_gpt import GPTBookClient,GPTSelectionError

class CredentialsTests(unittest.TestCase):
 def test_file_and_no_environment_mutation(self):
  with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True):
   p=Path(d)/'key';p.write_text('test-key\n')
   self.assertEqual(GPTBookClient(api_key_file=p)._api_key(),'test-key')
   self.assertNotIn('OPENAI_API_KEY',os.environ)
 def test_environment_wins(self):
  with patch.dict(os.environ,{'OPENAI_API_KEY':'env-key'}):
   self.assertEqual(GPTBookClient(api_key_file='/missing')._api_key(),'env-key')
 def test_bad_file_errors_never_contain_contents(self):
  for value in ('','secret value','x'*9000):
   with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{},clear=True):
    p=Path(d)/'key';p.write_text(value)
    with self.assertRaises(GPTSelectionError) as e:GPTBookClient(api_key_file=p).check_ready()
    self.assertNotIn('secret',str(e.exception));self.assertNotIn('xxx',str(e.exception))
 def test_missing_file(self):
  with patch.dict(os.environ,{},clear=True),self.assertRaises(GPTSelectionError):
   GPTBookClient(api_key_file='/missing').check_ready()
if __name__=='__main__':unittest.main()
