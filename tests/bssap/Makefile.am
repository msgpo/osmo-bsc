AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	-ggdb3 \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOSIGTRAN_CFLAGS) \
	$(COVERAGE_CFLAGS) \
	$(LIBOSMOMGCPCLIENT_CFLAGS) \
	$(NULL)

EXTRA_DIST = \
	bssap_test.ok \
	bssap_test.err \
	$(NULL)

noinst_PROGRAMS = \
	bssap_test \
	$(NULL)

bssap_test_SOURCES = \
	bssap_test.c \
	$(NULL)

bssap_test_LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOSIGTRAN_LIBS) \
	$(LIBOSMOMGCPCLIENT_LIBS) \
	$(NULL)

bssap_test_LDFLAGS = \
	$(top_builddir)/src/osmo-bsc/a_reset.o \
	$(top_builddir)/src/osmo-bsc/abis_rsl.o \
	$(top_builddir)/src/osmo-bsc/arfcn_range_encode.o \
	$(top_builddir)/src/osmo-bsc/bsc_api.o \
	$(top_builddir)/src/osmo-bsc/bsc_dyn_ts.o \
	$(top_builddir)/src/osmo-bsc/bsc_init.o \
	$(top_builddir)/src/osmo-bsc/bsc_rll.o \
	$(top_builddir)/src/osmo-bsc/bsc_subscriber.o \
	$(top_builddir)/src/osmo-bsc/chan_alloc.o \
	$(top_builddir)/src/osmo-bsc/gsm_04_08_utils.o \
	$(top_builddir)/src/osmo-bsc/gsm_04_80_utils.o \
	$(top_builddir)/src/osmo-bsc/gsm_data.o \
	$(top_builddir)/src/osmo-bsc/handover_cfg.o \
	$(top_builddir)/src/osmo-bsc/handover_logic.o \
	$(top_builddir)/src/osmo-bsc/net_init.o \
	$(top_builddir)/src/osmo-bsc/osmo_bsc_api.o \
	$(top_builddir)/src/osmo-bsc/osmo_bsc_bssap.o \
	$(top_builddir)/src/osmo-bsc/osmo_bsc_filter.o \
	$(top_builddir)/src/osmo-bsc/osmo_bsc_grace.o \
	$(top_builddir)/src/osmo-bsc/osmo_bsc_sigtran.o \
	$(top_builddir)/src/osmo-bsc/paging.o \
	$(top_builddir)/src/osmo-bsc/pcu_sock.o \
	$(top_builddir)/src/osmo-bsc/rest_octets.o \
	$(top_builddir)/src/osmo-bsc/system_information.o \
	-Wl,--wrap=bsc_grace_paging_request \
	$(NULL)

.PHONY: update_exp
update_exp:
	$(builddir)/bssap_test >$(srcdir)/bssap_test.ok 2>$(srcdir)/bssap_test.err
