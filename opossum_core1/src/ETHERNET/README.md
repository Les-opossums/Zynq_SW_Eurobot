# 🛠️ Patch BSP - Forcer un lien Ethernet à 100 Mbps (RTL8201F)

Ce correctif permet de forcer le lien Ethernet à **100 Mbps** en modifiant la fonction de détection du PHY Realtek dans le **Board Support Package (BSP)** généré par **Vitis**.

L'objectif est de supprimer les appels liés au Gigabit et de n'annoncer que les capacités **10/100 Mbps**.

---

## 1. Localiser le fichier à modifier

Dans votre **Workspace Vitis**, ouvrez le projet **Platform** (ou **BSP**) puis naviguez jusqu'au fichier suivant :

```text
[Nom_du_BSP]/
└── ps7_cortexa9_0/
    └── libsrc/
        └── lwip21x_v.../
            └── src/
                └── contrib/
                    └── ports/
                        └── xilinx/
                            └── netif/
                                └── xemacpsif_physpeed.c
```

---

## 2. Remplacer la fonction `get_Realtek_phy_speed`

Ouvrez le fichier **`xemacpsif_physpeed.c`**.

Recherchez la fonction :

```c
get_Realtek_phy_speed(...)
```

Puis remplacez-la **intégralement** par le code suivant :

```c
static u32_t get_Realtek_phy_speed(XEmacPs *xemacpsp, u32_t phy_addr)
{
	u16_t control;
	u16_t status;
	u32_t timeout_counter = 0;

	xil_printf("Start PHY autonegotiation (Patched for RTL8201F)\r\n");

	// 1. Annonce des capacités 10/100 Mbps (Full/Half Duplex)
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, &control);
	control |= IEEE_ASYMMETRIC_PAUSE_MASK;
	control |= IEEE_PAUSE_MASK;
	control |= ADVERTISE_100;
	control |= ADVERTISE_10;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_AUTONEGO_ADVERTISE_REG, control);

	// /!\ Patch : L'annonce du Gigabit a été supprimée ici pour éviter de perturber le PHY.

	// 2. Redémarrage de l'auto-négociation
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_AUTONEGOTIATE_ENABLE;
	control |= IEEE_STAT_AUTONEGOTIATE_RESTART;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	// 3. Reset du PHY
	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
	control |= IEEE_CTRL_RESET_MASK;
	XEmacPs_PhyWrite(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, control);

	// Attente de la fin du reset
	while (1) {
		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_CONTROL_REG_OFFSET, &control);
		if (control & IEEE_CTRL_RESET_MASK)
			continue;
		else
			break;
	}

	XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);

	xil_printf("Waiting for PHY to complete autonegotiation.\r\n");

	// 4. Attente de la fin de l'auto-négociation
	while (!(status & IEEE_STAT_AUTONEGOTIATE_COMPLETE)) {
		sleep(1);
		timeout_counter++;

		if (timeout_counter == 30) {
			xil_printf("Auto negotiation error \r\n");
			return XST_FAILURE;
		}

		XEmacPs_PhyRead(xemacpsp, phy_addr, IEEE_STATUS_REG_OFFSET, &status);
	}

	xil_printf("autonegotiation complete \r\n");

	// 5. Patch : On ignore la lecture du registre spécifique Gigabit
	// et on force le retour à 100 Mbps.
	return 100;
}
```

---

## 3. Recompiler le projet

> **Attention**
>
> Vitis ne recompile pas toujours automatiquement le BSP lorsqu'il n'y a pas eu de modification matérielle.

Effectuez les étapes suivantes :

1. Sauvegardez le fichier.
2. Faites un clic droit sur le projet **Platform/BSP** puis sélectionnez **Build Project**.
3. Faites un clic droit sur votre projet **Application** puis sélectionnez **Build Project**.

---

## Vérification

Au prochain démarrage de la carte, la console série devrait afficher :

```text
link speed for phy address 1: 100
```

Si ce message apparaît, le patch a bien été appliqué et le lien Ethernet est désormais forcé à **100 Mbps**.