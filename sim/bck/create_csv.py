import pandas as pd
import re
import os

files = ['LM311_Cpar_sweep.txt', 'LM311_Ra_sweep.txt', 'LM311_Rpot_sweep.txt']

for file in files:
    if not os.path.exists(file):
        continue
        
    with open(file, 'r') as f:
        lines = f.readlines()
    
    steps_data = {}
    current_step = None
    
    # Načítanie dát
    for line in lines:
        line = line.strip()
        if line.startswith('time\t'):
            headers = line.split('\t')
        elif line.startswith('Step Information:'):
            match = re.search(r'Step Information:\s*(.*?)\s*\(Step:\s*(\d+)', line)
            current_step = f"Step{match.group(2)}_{match.group(1)}" if match else line
            steps_data[current_step] = {h: [] for h in headers}
        elif line and current_step:
            parts = line.split()
            if len(parts) == len(headers):
                for i, h in enumerate(headers):
                    # Rovno prevádzame na float pre správne spájanie a interpoláciu
                    steps_data[current_step][h].append(float(parts[i]))
                    
    if not steps_data:
        continue

    # Vytvorenie zoznamu tabuliek s premenovanými stĺpcami
    dfs = []
    for step_name, data in steps_data.items():
        df = pd.DataFrame(data)
        df.columns = [f"{c}_{step_name}" if c != 'time' else 'time' for c in df.columns]
        dfs.append(df)
    
    # Spájanie podľa stĺpca 'time' (outer join)
    result_df = dfs[0]
    for df in dfs[1:]:
        result_df = pd.merge(result_df, df, on='time', how='outer')
        
    # Zoradenie podľa času, interpolácia chýbajúcich hodnôt a vyplnenie okrajov
    result_df = result_df.sort_values('time').reset_index(drop=True)
    result_df = result_df.interpolate(method='linear').bfill().ffill()
    
    out_name = file.replace('.txt', '.csv')
    result_df.to_csv(out_name, index=False)
    print(f"Uložené: {out_name} (zlúčené podľa času bez prázdnych riadkov)")