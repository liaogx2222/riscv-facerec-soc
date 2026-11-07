"""Bind signoff to the actual static PLL and system frequency parameters."""
from pathlib import Path
import re

def configuration(rtl_root):
    rtl_root=Path(rtl_root)
    top=(rtl_root/'board/facerec_top.v').read_text(encoding='utf-8')
    pll=(rtl_root/'board/pll.v').read_text(encoding='utf-8')
    def value(name):
        match=re.search(r'\.'+name+r'\s*\(\s*([\d_.]+)\s*\)',pll)
        assert match,'Unsupported PLL setting: '+name
        return float(match[1].replace('_',''))
    declared=int(re.search(r'parameter\s+integer\s+CLK_HZ\s*=\s*([\d_]+)',top)[1].replace('_',''))
    assert re.search(r'\.INTERNAL_FB\s*\(\s*"CLKOUTF"\s*\)',pll),'Unsupported PLL feedback mode'
    assert re.search(r'\.CLK_FREQ\s*\(\s*CLK_HZ\s*\)',top),'SoC/UART clock parameter not linked to board'
    # Logos2 GPLL limits from the bundled official IP divider calculator:
    # input 10..800MHz, PFD 10..450MHz, VCO 600..1200MHz.
    clkin=value('CLKIN_FREQ')*1e6
    pfd=clkin/value('STATIC_RATIOI')
    vco=pfd*value('STATIC_RATIOM')*value('STATIC_RATIOF')
    assert 10e6<=clkin<=800e6,'GPLL reference clock outside 10..800MHz'
    assert 10e6<=pfd<=450e6,'GPLL PFD outside official 10..450MHz range'
    assert 600e6<=vco<=1200e6,'GPLL VCO outside official 600..1200MHz range'
    actual=vco/value('STATIC_RATIO0')
    assert abs(actual-declared)<1,'PLL and system clock parameter disagree'
    result=dict(clock_hz=declared,pll_output_hz=actual,pll_vco_hz=vco,pll_pfd_hz=pfd,
                pll_physical_ranges_valid=True,period_ns=1e9/actual)
    cnn=re.search(r'parameter\s+integer\s+CNN_CLK_HZ\s*=\s*([\d_]+)',top)
    if cnn:
        cnn_hz=int(cnn[1].replace('_',''));cnn_actual=vco/value('STATIC_RATIO1')
        assert abs(cnn_actual-cnn_hz)<1,'CNN PLL output and CNN_CLK_HZ disagree'
        assert re.search(r'\.cnn_clk\s*\(\s*clk_cnn\s*\)',top),'CNN clock not connected'
        result.update(cnn_clock_hz=cnn_hz,cnn_pll_output_hz=cnn_actual,cnn_period_ns=1e9/cnn_actual)
    return result

def constrained_configuration(rtl_root,fdc):
    result=configuration(rtl_root)
    text=Path(fdc).read_text(encoding='utf-8')
    match=re.search(r'create_clock\s+-name\s+clk_sys\s+-period\s+([\d.]+)',text)
    assert match,'Missing clk_sys constraint'
    result['constraint_period_ns']=float(match[1])
    result['consistent']=abs(float(match[1])-result['period_ns'])<0.00001
    if 'cnn_clock_hz' in result:
        cnn=re.search(r'create_clock\s+-name\s+clk_cnn\s+-period\s+([\d.]+)',text)
        result['cnn_constraint_period_ns']=float(cnn[1]) if cnn else None
        result['consistent']=result['consistent'] and bool(cnn) and abs(float(cnn[1])-result['cnn_period_ns'])<0.00001
    return result
