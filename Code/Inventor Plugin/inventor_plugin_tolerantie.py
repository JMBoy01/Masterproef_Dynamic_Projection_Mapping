import os
import json
import tkinter as tk
from tkinter import messagebox
from win32com.client import Dispatch, gencache

class InventorTolerancePlugin:
    def __init__(self, root):
        self.root = root
        self.root.title("Inventor Tolerance Plugin")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.mod = gencache.EnsureModule('{D98A091D-3A0F-4C3E-B36E-61F62068D488}', 0, 1, 0)

        self.app = Dispatch("Inventor.Application")
        self.app = self.mod.Application(self.app)

        self.doc = self.app.ActiveDocument
        self.part_name = os.path.basename(self.doc.FullFileName)
        self.json_path = "./json/"
        self.data_file = self.json_path + self.part_name + ".json"
        self.tolerance_data = self.load_tolerance_data()
        
        self.tolerance_entries = []
        
        self.frame = tk.Frame(root)
        self.frame.pack(padx=10, pady=10)
        
        self.add_tolerance_button = tk.Button(self.frame, text="+ Tolerantie", command=self.add_tolerance_row)
        self.add_tolerance_button.grid(row=0, column=0, columnspan=3, pady=5)
        
        self.load_existing_tolerances()
        
        self.apply_button = tk.Button(self.frame, text="Toepassen", command=self.apply_tolerances)
        self.apply_button.grid(row=999, column=0, columnspan=3, pady=5)
    
    def load_tolerance_data(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, "r") as f:
                return json.load(f)
        return {}

    def save_tolerance_data(self):
        with open(self.data_file, "w") as f:
            json.dump(self.tolerance_data, f, indent=4)
    
    def add_tolerance_row(self, tolerance=None, dimensions=None, active=True):
        row = len(self.tolerance_entries) + 1
        tol_var = tk.StringVar(value=tolerance if tolerance else "")
        dims_var = tk.StringVar(value=", ".join(dimensions) if dimensions else "")
        active_var = tk.BooleanVar(value=active)

        # Voeg de invoervelden en de checkbox toe
        tk.Entry(self.frame, textvariable=tol_var).grid(row=row, column=0)
        tk.Entry(self.frame, textvariable=dims_var).grid(row=row, column=1)
        tk.Checkbutton(self.frame, variable=active_var).grid(row=row, column=2)
        
        # Voeg een delete-knop toe
        delete_button = tk.Button(self.frame, text="Verwijder", command=lambda: self.delete_tolerance_row(row, tol_var, dims_var, active_var))
        delete_button.grid(row=row, column=3)  # Plaats de delete-knop in kolom 3

        self.tolerance_entries.append((tol_var, dims_var, active_var))

    def delete_tolerance_row(self, row, tol_var, dims_var, active_var):
        # Verwijder de rij uit de UI
        for widget in self.frame.grid_slaves():
            if int(widget.grid_info()["row"]) == row:
                widget.grid_remove()  # Verberg de widget

        # Verwijder de entry uit de lijst
        self.tolerance_entries.remove((tol_var, dims_var, active_var))

        # Verwijder de entry uit data
        self.tolerance_data.pop(tol_var.get(), None)
        print(tol_var.get())
    
    def load_existing_tolerances(self):
        for tol, data in self.tolerance_data.items():
            self.add_tolerance_row(tol, data['dimensions'], data['active'])
    
    def apply_tolerances(self):
        # https://help.autodesk.com/view/INVNTOR/2022/ENU/?guid=DocumentTypeEnum
        if self.doc.DocumentType != 12290:
            messagebox.showinfo("Error", "Open document is geen part (.ipt)!")
            return

        try:
            # Cast naar PartDocument: https://forums.autodesk.com/t5/inventor-programming-ilogic/importing-an-assembly-document-into-python/td-p/11348001#:~:text=oAssemblyPartDoc%20%3D%20mod.AssemblyDocument(oPartDoc)
            part_doc = self.mod.PartDocument(self.doc)
        except Exception as e:
            messagebox.showerror("Fout", f"Fout bij het casten naar PartDocument: {e}")
            return

        for tol_var, dims_var, active_var in self.tolerance_entries:
            try:
                tol = float(tol_var.get())
            except Exception as e:
                    messagebox.showerror("Fout", f"Tolerantie is geen getal: {e}")

            dims = [d.strip() for d in dims_var.get().split(",")]
            active = active_var.get()
            self.tolerance_data[str(tol)] = {"dimensions": dims, "active": active}

            print(self.tolerance_data)
            
            for dim_name in dims:
                try:
                    param = part_doc.ComponentDefinition.Parameters.Item(dim_name)

                    if active: # Delen door 10 want interpreteerd dit als cm en niet mm zoals in inventor
                        param.Value = float(param.Value) + tol/10
                    else:
                        param.Value = float(param.Value) - tol/10

                    part_doc.Update()
                except Exception as e:
                    messagebox.showerror("Fout", f"Kon tolerantie niet toepassen op {dim_name}: {e}")
        
        self.save_tolerance_data()
        # messagebox.showinfo("Succes", "Toleranties toegepast en opgeslagen!")

    def on_close(self):
        if messagebox.askokcancel("Afsluiten", "Weet je zeker dat je het programma wilt afsluiten?"):
            # Check if most recent data is saved with json dumps -> order independant
            saved_data = self.load_tolerance_data() or {}
            if json.dumps(saved_data, sort_keys=True) != json.dumps(self.tolerance_data, sort_keys=True):
                result = messagebox.askyesnocancel("Opslaan?", "De huidige tolerantie instelling zijn nog niet opgeslagen, wil je ze opslaan?")
                if result == True:
                    self.save_tolerance_data()
                    self.root.destroy()
                elif result == False:
                    self.root.destroy()
                else:
                    pass
            else:
                self.root.destroy()
            

if __name__ == "__main__":
    root = tk.Tk()
    app = InventorTolerancePlugin(root)
    root.mainloop()
