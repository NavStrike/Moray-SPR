from functions.adminData import accessData

p2 = accessData("substances.json")
# Sustancias
listSubstances = p2.data.get("name")
print(listSubstances)