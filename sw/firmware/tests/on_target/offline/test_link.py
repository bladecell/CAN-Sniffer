import unittest
from check_link import verify_disassembly


class LinkEvidenceTests(unittest.TestCase):
    def test_requires_combined_production_wrapper_and_sdk_calls(self):
        text = """42000000 <start_sim_task(CanDriver*)>:
42000000: 123456 call8 42000020 <__wrap_xTaskCreatePinnedToCore>

42000020 <__wrap_xTaskCreatePinnedToCore>:
42000020: 123456 call8 42000100 <legacyTaskCreationShouldFail()>
42000024: 123456 l32r a8, 42000030 (40380000 <xTaskCreatePinnedToCore>)
42000028: 123456 callx8 a8

"""
        verify_disassembly(text)
        for bad in (
            text.replace("<__wrap_xTaskCreatePinnedToCore>\n", "<bypass>\n"),
            text.replace("<xTaskCreatePinnedToCore>", "<wrong_sdk_function>"),
            text.replace("callx8", "mov.n"),
            text.replace("<legacyTaskCreationShouldFail()>", "<missing>"),
        ):
            with self.assertRaises(ValueError):
                verify_disassembly(bad)


if __name__ == "__main__":
    unittest.main()
