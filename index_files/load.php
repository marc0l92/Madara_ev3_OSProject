var mediaWikiLoadStart=(new Date()).getTime(),mwPerformance=(window.performance&&performance.mark)?performance:{mark:function(){}};mwPerformance.mark('mwLoadStart');function isCompatible(ua){if(ua===undefined){ua=navigator.userAgent;}return!((ua.indexOf('MSIE')!==-1&&parseFloat(ua.split('MSIE')[1])<8)||(ua.indexOf('Firefox/')!==-1&&parseFloat(ua.split('Firefox/')[1])<3)||(ua.indexOf('Opera/')!==-1&&(ua.indexOf('Version/')===-1?parseFloat(ua.split('Opera/')[1])<10:parseFloat(ua.split('Version/')[1])<12))||(ua.indexOf('Opera ')!==-1&&parseFloat(ua.split(' Opera ')[1])<10)||ua.match(/BlackBerry[^\/]*\/[1-5]\./)||ua.match(/webOS\/1\.[0-4]/)||ua.match(/PlayStation/i)||ua.match(/SymbianOS|Series60/)||ua.match(/NetFront/)||ua.match(/Opera Mini/)||ua.match(/S40OviBrowser/)||ua.match(/MeeGo/)||(ua.match(/Glass/)&&ua.match(/Android/)));}(function(){if(!isCompatible()){document.documentElement.className=document.documentElement.className.replace(/(^|\s)client-js(\s|$)/,'$1client-nojs$2');return;}
function startUp(){mw.config=new mw.Map(true);mw.loader.addSource({"local":"/mediawiki-1.26.0/load.php"});mw.loader.register([["site","JkyXZhA2"],["noscript","HiqG0+nW",[],"noscript"],["filepage","qpgjCCxd"],["user.groups","M6DDUJ3n",[],"user"],["user","XDpfZp7/",[],"user"],["user.cssprefs","64Nx0RWw",[],"private"],["user.defaults","hY5NkNwh"],["user.options","+JoudQIu",[6],"private"],["user.tokens","fekXsbJP",[],"private"],["mediawiki.language.data","pJtq8AIz",[168]],["mediawiki.skinning.elements","zD3PAg5O"],["mediawiki.skinning.content","9ETFO6Qj"],["mediawiki.skinning.interface","4RgZb72Y"],["mediawiki.skinning.content.parsoid","DbVGxLTH"],["mediawiki.skinning.content.externallinks","MXJj8Hwd"],["jquery.accessKeyLabel","UwQgjmri",[25,129]],["jquery.appear","zX6mU4VP"],["jquery.arrowSteps","j8fRlxo/"],["jquery.async","+dhXCBlu"],["jquery.autoEllipsis","Qy6wGkM+",[37]],["jquery.badge","8dAVFtaC",[165]],["jquery.byteLength","4TZZTLpp"],["jquery.byteLimit","fJU4V1zg",[21]],[
"jquery.checkboxShiftClick","36U3h9SR"],["jquery.chosen","JXXCTyk5"],["jquery.client","x1/N5wPW"],["jquery.color","xml2+s7I",[27]],["jquery.colorUtil","dWWUjMo1"],["jquery.confirmable","rX7e6vod",[169]],["jquery.cookie","aL7aYIKr"],["jquery.expandableField","892kLpgO"],["jquery.farbtastic","cFtVe4VY",[27]],["jquery.footHovzer","lGAKShzl"],["jquery.form","949fGnJ5"],["jquery.fullscreen","LTf/BuuS"],["jquery.getAttrs","2IxRIySz"],["jquery.hidpi","zqHUaKPr"],["jquery.highlightText","mXYC28kg",[227,129]],["jquery.hoverIntent","o8QxIdo4"],["jquery.i18n","PvIPLNM0",[167]],["jquery.localize","yEOkvjzl"],["jquery.makeCollapsible","eWPJGWF6"],["jquery.mockjax","UDjJhcuA"],["jquery.mw-jump","htrV001N"],["jquery.mwExtension","IG/m4gj0"],["jquery.placeholder","MGytjMwK"],["jquery.qunit","eukTii1O"],["jquery.qunit.completenessTest","+kjK/dJZ",[46]],["jquery.spinner","WYXsLUWw"],["jquery.jStorage","9l8Lf1tH",[93]],["jquery.suggestions","pz0co1Ok",[37]],["jquery.tabIndex","jx6ULbfN"],[
"jquery.tablesorter","nF8ec9LB",[227,129,170]],["jquery.textSelection","Cq4y7pQk",[25]],["jquery.throttle-debounce","s+D2eeRG"],["jquery.validate","EcTaYFzu"],["jquery.xmldom","LVOL9C95"],["jquery.tipsy","SN2wQjJE"],["jquery.ui.core","GjbQxhWo",[59],"jquery.ui"],["jquery.ui.core.styles","K3LXA0uE",[],"jquery.ui"],["jquery.ui.accordion","XRDSAstM",[58,78],"jquery.ui"],["jquery.ui.autocomplete","8wUXLl3E",[67],"jquery.ui"],["jquery.ui.button","YasOFb6a",[58,78],"jquery.ui"],["jquery.ui.datepicker","1oYobvSH",[58],"jquery.ui"],["jquery.ui.dialog","TfD6qgnq",[62,65,69,71],"jquery.ui"],["jquery.ui.draggable","3OTVRpTk",[58,68],"jquery.ui"],["jquery.ui.droppable","lqhds+LB",[65],"jquery.ui"],["jquery.ui.menu","pJphi8JM",[58,69,78],"jquery.ui"],["jquery.ui.mouse","ZX7joGbm",[78],"jquery.ui"],["jquery.ui.position","njpPJ8BR",[],"jquery.ui"],["jquery.ui.progressbar","v9yc9CaW",[58,78],"jquery.ui"],["jquery.ui.resizable","2DmsZq60",[58,68],"jquery.ui"],["jquery.ui.selectable","fdRulLfD",[58,68],
"jquery.ui"],["jquery.ui.slider","Fjxex9y3",[58,68],"jquery.ui"],["jquery.ui.sortable","HLz+8e3/",[58,68],"jquery.ui"],["jquery.ui.spinner","dtHBWyZi",[62],"jquery.ui"],["jquery.ui.tabs","FXyzLFlK",[58,78],"jquery.ui"],["jquery.ui.tooltip","qK3zWj4j",[58,69,78],"jquery.ui"],["jquery.ui.widget","RBmab0E3",[],"jquery.ui"],["jquery.effects.core","3goJpGhd",[],"jquery.ui"],["jquery.effects.blind","t0VZElsv",[79],"jquery.ui"],["jquery.effects.bounce","4jYs6x+z",[79],"jquery.ui"],["jquery.effects.clip","kaYhTX8L",[79],"jquery.ui"],["jquery.effects.drop","ORt8uzzO",[79],"jquery.ui"],["jquery.effects.explode","WlWBHGpl",[79],"jquery.ui"],["jquery.effects.fade","VTEzk56k",[79],"jquery.ui"],["jquery.effects.fold","3kXp/gc2",[79],"jquery.ui"],["jquery.effects.highlight","DZG7AYi4",[79],"jquery.ui"],["jquery.effects.pulsate","FH+UzeOy",[79],"jquery.ui"],["jquery.effects.scale","Na0+YYB9",[79],"jquery.ui"],["jquery.effects.shake","j1KZ6O8i",[79],"jquery.ui"],["jquery.effects.slide","glDGNPTl",[79],
"jquery.ui"],["jquery.effects.transfer","X8lgBldn",[79],"jquery.ui"],["json","ufmEBzPS",[],null,null,"return!!(window.JSON\u0026\u0026JSON.stringify\u0026\u0026JSON.parse);"],["moment","maDjB6oO"],["mediawiki.apihelp","jJ/x+onB",[119]],["mediawiki.template","XlsmWx78"],["mediawiki.template.mustache","Dv4kcm5V",[96]],["mediawiki.template.regexp","mOTtBA4w",[96]],["mediawiki.apipretty","wTUASHh/"],["mediawiki.api","PB3msbWY",[145,8]],["mediawiki.api.category","xiMxcBxc",[134,100]],["mediawiki.api.edit","UCCNEieL",[134,100]],["mediawiki.api.login","kb1V+3It",[100]],["mediawiki.api.options","pb5bypX+",[100]],["mediawiki.api.parse","V2VGXQa2",[100]],["mediawiki.api.upload","fydlE1MU",[227,93,102]],["mediawiki.api.watch","lgNHE7Ms",[100]],["mediawiki.content.json","l0rZ+mik"],["mediawiki.confirmCloseWindow","xLYsVUNB"],["mediawiki.debug","IYF09CoZ",[32,57]],["mediawiki.debug.init","vjFeBTzq",[110]],["mediawiki.feedback","Mq2YlSa6",[134,125,229]],["mediawiki.feedlink","vePPr5WD"],[
"mediawiki.filewarning","j6kKOSE7",[229]],["mediawiki.ForeignApi","Ejh8WGgE",[116]],["mediawiki.ForeignApi.core","TQniRPRv",[100,228]],["mediawiki.helplink","yfOx+bdR"],["mediawiki.hidpi","FBqq6Lav",[36],null,null,"return'srcset'in new Image();"],["mediawiki.hlist","B85pOqQ8",[25]],["mediawiki.htmlform","MEat0FGF",[22,129]],["mediawiki.htmlform.styles","vngefzm+"],["mediawiki.htmlform.ooui.styles","o4m4oaBO"],["mediawiki.icon","s+8ax34a"],["mediawiki.inspect","waujenE+",[21,93,129]],["mediawiki.messagePoster","AvNDKSKJ",[100,228]],["mediawiki.messagePoster.wikitext","O8JaLpeG",[102,125]],["mediawiki.notification","MdoEqNpq",[177]],["mediawiki.notify","NyQd48Zs"],["mediawiki.RegExp","5l8Z7rg/"],["mediawiki.pager.tablePager","vuqicf6k"],["mediawiki.searchSuggest","0c91tLBQ",[35,45,50,100]],["mediawiki.sectionAnchor","YHBAuyeG"],["mediawiki.storage","D3H3gZEG"],["mediawiki.Title","dy0/0S3H",[21,145]],["mediawiki.Upload","rwzlkmXu",[106]],["mediawiki.ForeignUpload","8j1MVRVE",[115,135]],[
"mediawiki.ForeignStructuredUpload","Dn447w95",[136]],["mediawiki.Upload.Dialog","KestaTbb",[139]],["mediawiki.Upload.BookletLayout","E7ke/tBb",[135,169,229]],["mediawiki.ForeignStructuredUpload.BookletLayout","M8Yid3O9",[137,139,224,223]],["mediawiki.toc","sPllYIUh",[146]],["mediawiki.Uri","LwGMEouU",[145,98]],["mediawiki.user","XLzGYk8d",[100,146,7]],["mediawiki.userSuggest","gW6Mefcy",[50,100]],["mediawiki.util","dwFMm2Oo",[15,128]],["mediawiki.cookie","hcnuftcb",[29]],["mediawiki.toolbar","GOQmjGH1"],["mediawiki.experiments","8hbSRY1X"],["mediawiki.action.edit","1wZdiPIo",[22,53,150]],["mediawiki.action.edit.styles","7BDqgMSz"],["mediawiki.action.edit.collapsibleFooter","Yx+9NiIf",[41,146,123]],["mediawiki.action.edit.preview","d06lI1Nj",[33,48,53,155,100,169]],["mediawiki.action.edit.stash","OyHxdHUb",[35,100]],["mediawiki.action.history","4cIpSmV9"],["mediawiki.action.history.diff","uyi+vr1d"],["mediawiki.action.view.dblClickEdit","qNXXJb2r",[177,7]],[
"mediawiki.action.view.metadata","6Wq6J8Ua"],["mediawiki.action.view.categoryPage.styles","fAx7koDn"],["mediawiki.action.view.postEdit","WGaD6PVc",[146,169,96]],["mediawiki.action.view.redirect","6YzogVLv",[25]],["mediawiki.action.view.redirectPage","aLjz6jl+"],["mediawiki.action.view.rightClickEdit","rOlY+/Yv"],["mediawiki.action.edit.editWarning","TXxEkNO6",[53,109,169]],["mediawiki.action.view.filepage","LZoRmRJC"],["mediawiki.language","JfdZIbs+",[166,9]],["mediawiki.cldr","CfnYOKj6",[167]],["mediawiki.libs.pluralruleparser","x7KC5U6/"],["mediawiki.language.init","+C/jzRZJ"],["mediawiki.jqueryMsg","EjtXgyaf",[227,165,145,7]],["mediawiki.language.months","e5l2m9RG",[165]],["mediawiki.language.names","PEZBZtR9",[168]],["mediawiki.language.specialCharacters","lfy9K43n",[165]],["mediawiki.libs.jpegmeta","+iavpO53"],["mediawiki.page.gallery","REwAIBc3",[54,175]],["mediawiki.page.gallery.styles","X0otr6S1"],["mediawiki.page.ready","3N7CxG2g",[15,23,41,43,45]],["mediawiki.page.startup",
"Ussy5dR6",[145]],["mediawiki.page.patrol.ajax","0QedfoDF",[48,134,100,177]],["mediawiki.page.watch.ajax","ZWLFHOP3",[107,177]],["mediawiki.page.image.pagination","+PVXVnzQ",[48,142]],["mediawiki.special","d15rCwZj"],["mediawiki.special.block","lUV3Ndbl",[145]],["mediawiki.special.changeemail","AeVF7l9j",[145]],["mediawiki.special.changeslist","mYXafGnB"],["mediawiki.special.changeslist.legend","RyfGxrL6"],["mediawiki.special.changeslist.legend.js","0JJNQeTN",[41,146]],["mediawiki.special.changeslist.enhanced","lzBn3abk"],["mediawiki.special.edittags","WrqlbxdW",[24]],["mediawiki.special.edittags.styles","Ix0aTNve"],["mediawiki.special.import","W6OrvqLV"],["mediawiki.special.movePage","YjirbKAt",[221]],["mediawiki.special.movePage.styles","6yvAOlrr"],["mediawiki.special.pageLanguage","bu+RcTjT"],["mediawiki.special.pagesWithProp","9z3VtlS7"],["mediawiki.special.preferences","86rXAXiy",[109,165,127]],["mediawiki.special.recentchanges","+dJ8qOht",[181]],["mediawiki.special.search",
"NcZ/CeBZ"],["mediawiki.special.undelete","LEkML68g"],["mediawiki.special.upload","1ua0G6h7",[48,134,100,109,169,173,96]],["mediawiki.special.userlogin.common.styles","Pen39ESk"],["mediawiki.special.userlogin.signup.styles","iL2YZen6"],["mediawiki.special.userlogin.login.styles","ZCcWQnZl"],["mediawiki.special.userlogin.signup.js","KhY9M3YC",[54,100,169]],["mediawiki.special.unwatchedPages","B5D36Ms0",[134,107]],["mediawiki.special.javaScriptTest","3fZcrymO",[142]],["mediawiki.special.version","UuHIl494"],["mediawiki.legacy.config","OZ56i8EU"],["mediawiki.legacy.commonPrint","llVanB9E"],["mediawiki.legacy.protect","nUKeqbd3",[22]],["mediawiki.legacy.shared","yAyIqCgi"],["mediawiki.legacy.oldshared","AvfQTJv5"],["mediawiki.legacy.wikibits","5/KBF9Z5",[145]],["mediawiki.ui","F2u4k/kr"],["mediawiki.ui.checkbox","UJCZv99X"],["mediawiki.ui.radio","CgUoKdo0"],["mediawiki.ui.anchor","zITcby2O"],["mediawiki.ui.button","sXVTKZCW"],["mediawiki.ui.input","7VFcdieP"],["mediawiki.ui.icon",
"sOIkvb06"],["mediawiki.ui.text","6uHZgVYw"],["mediawiki.widgets","PPCIRITU",[19,22,115,134,224,222]],["mediawiki.widgets.styles","LWw21wsP"],["mediawiki.widgets.DateInputWidget","Wg6mAp/a",[94,229]],["mediawiki.widgets.CategorySelector","BHl7mat3",[100,229]],["mediawiki.widgets.UserInputWidget","PuTtgsIf",[229]],["es5-shim","MqZTMhhY",[],null,null,"return(function(){'use strict';return!this\u0026\u0026!!Function.prototype.bind;}());"],["dom-level2-shim","HTapryIh",[],null,null,"return!!window.Node;"],["oojs","/YhcygYR",[226,93]],["oojs-ui","hs++W8bN",[228,230,231,232,233]],["oojs-ui.styles","HL37cG9V"],["oojs-ui.styles.icons","8LbLeGv7"],["oojs-ui.styles.indicators","unZRGXBr"],["oojs-ui.styles.textures","W+45eQc5"],["oojs-ui.styles.icons-accessibility","LZoitlZC"],["oojs-ui.styles.icons-alerts","Vg0ID9yb"],["oojs-ui.styles.icons-content","mOA6byVu"],["oojs-ui.styles.icons-editing-advanced","NXiSgBX+"],["oojs-ui.styles.icons-editing-core","yGvLOgmP"],[
"oojs-ui.styles.icons-editing-list","VKmTacEi"],["oojs-ui.styles.icons-editing-styling","yU3OfMEH"],["oojs-ui.styles.icons-interactions","OCJUQvc+"],["oojs-ui.styles.icons-layout","9x/SYG8B"],["oojs-ui.styles.icons-location","xAXAlYF3"],["oojs-ui.styles.icons-media","KMGVTEpR"],["oojs-ui.styles.icons-moderation","X7NeZEuU"],["oojs-ui.styles.icons-movement","N0pDi9aj"],["oojs-ui.styles.icons-user","bTV6I1x+"],["oojs-ui.styles.icons-wikimedia","oZcKvd7c"],["skins.cologneblue","LkI5lbEh"],["skins.modern","XntCohTP"],["skins.monobook.styles","KIssCOS+"],["skins.vector.styles","y5ErAKSu"],["skins.vector.styles.responsive","olyJOP3K"],["skins.vector.js","YrdA+UTJ",[51,54]]]);;mw.config.set({"wgLoadScript":"/mediawiki-1.26.0/load.php","debug":!1,"skin":"vector","stylepath":"/mediawiki-1.26.0/skins","wgUrlProtocols":
"bitcoin\\:|ftp\\:\\/\\/|ftps\\:\\/\\/|geo\\:|git\\:\\/\\/|gopher\\:\\/\\/|http\\:\\/\\/|https\\:\\/\\/|irc\\:\\/\\/|ircs\\:\\/\\/|magnet\\:|mailto\\:|mms\\:\\/\\/|news\\:|nntp\\:\\/\\/|redis\\:\\/\\/|sftp\\:\\/\\/|sip\\:|sips\\:|sms\\:|ssh\\:\\/\\/|svn\\:\\/\\/|tel\\:|telnet\\:\\/\\/|urn\\:|worldwind\\:\\/\\/|xmpp\\:|\\/\\/","wgArticlePath":"/mediawiki-1.26.0/index.php/$1","wgScriptPath":"/mediawiki-1.26.0","wgScriptExtension":".php","wgScript":"/mediawiki-1.26.0/index.php","wgSearchType":null,"wgVariantArticlePath":!1,"wgActionPaths":{},"wgServer":"http://localhost","wgServerName":"localhost","wgUserLanguage":"en","wgContentLanguage":"en","wgTranslateNumerals":!0,"wgVersion":"1.26.0","wgEnableAPI":!0,"wgEnableWriteAPI":!0,"wgMainPageTitle":"Main Page","wgFormattedNamespaces":{"-2":"Media","-1":"Special","0":"","1":"Talk","2":"User","3":"User talk","4":"Robot Project","5":"Robot Project talk","6":"File","7":"File talk","8":"MediaWiki","9":"MediaWiki talk","10":"Template","11"
:"Template talk","12":"Help","13":"Help talk","14":"Category","15":"Category talk"},"wgNamespaceIds":{"media":-2,"special":-1,"":0,"talk":1,"user":2,"user_talk":3,"robot_project":4,"robot_project_talk":5,"file":6,"file_talk":7,"mediawiki":8,"mediawiki_talk":9,"template":10,"template_talk":11,"help":12,"help_talk":13,"category":14,"category_talk":15,"image":6,"image_talk":7,"project":4,"project_talk":5},"wgContentNamespaces":[0],"wgSiteName":"Robot Project","wgDBname":"my_wiki","wgExtraSignatureNamespaces":[],"wgAvailableSkins":{"cologneblue":"CologneBlue","modern":"Modern","monobook":"MonoBook","vector":"Vector","fallback":"Fallback","apioutput":"ApiOutput"},"wgExtensionAssetsPath":"/mediawiki-1.26.0/extensions","wgCookiePrefix":"my_wiki","wgCookieDomain":"","wgCookiePath":"/","wgCookieExpiration":15552000,"wgResourceLoaderMaxQueryLength":2000,"wgCaseSensitiveNamespaces":[],"wgLegalTitleChars":" %!\"$&'()*,\\-./0-9:;=?@A-Z\\\\\\^_`a-z~+\\u0080-\\uFFFF","wgResourceLoaderStorageVersion":
1,"wgResourceLoaderStorageEnabled":!1,"wgResourceLoaderLegacyModules":["mediawiki.legacy.wikibits"],"wgForeignUploadTargets":[],"wgEnableUploads":!0});window.RLQ=window.RLQ||[];while(RLQ.length){RLQ.shift()();}window.RLQ={push:function(fn){fn();}};}var script=document.createElement('script');script.src="/mediawiki-1.26.0/load.php?debug=false&lang=en&modules=jquery%2Cmediawiki&only=scripts&skin=vector&version=OKT0CnuW";script.onload=script.onreadystatechange=function(){if(!script.readyState||/loaded|complete/.test(script.readyState)){script.onload=script.onreadystatechange=null;script=null;startUp();}};document.getElementsByTagName('head')[0].appendChild(script);}());